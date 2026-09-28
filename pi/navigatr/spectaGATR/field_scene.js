// field_scene.js
// The 3D field. Static geometry, nominal landmarks and tag mounts come from
// hello.fields, and so does the planning data the Brain is served: the
// boundary and the collision boxes (fixed obstacles in their own frame,
// landmark boxes in the landmark frame, drawn at the estimate when observed,
// else at nominal). The robot and its trail come from state messages, the
// camera mounts, estimated landmark bodies and the Brain's reported path
// from diag messages, and the destination/target overlays from telemetry.
// The robot is drawn with the running Brain profile's footprint when one is
// applied, else with the display-only RobotBody. Scene axes are the field
// axes one to one: THREE.Object3D.DEFAULT_UP is +z, so field x is scene x
// (audience right), field y is scene y (toward the 0-degree wall), field z
// is scene z (up). No mirroring anywhere, so the right-handed field stays
// right-handed on screen; the wall labels come from the landmark ids in
// hello (blue goals at large x), which makes a handedness mistake visible
// instead of silent.
//
// Transform chains are applied explicitly with Object3D nesting:
//   field <- landmark (nominal or estimate, planar) <- tag surface (mount)
//   field <- robot origin (planar pose, tilt when attitude is measured)
//         <- camera engineering frame (T_robot_camera)
//   field <- odometry frame (field_from_odom) <- trail points
// Mesh origins never stand in for landmark or robot origins: bodies are
// translated inside their group so the group origin is the configured one.
//
// Nothing here runs on message arrival: the page's animation loop calls the
// apply* methods with the newest stored messages, then render(). The drawn
// robot pose is the presentation-only smoothed one (smoothing.js); the
// trail, readouts and tooltips use exact received values.

import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { DEG, rotationOf, fmt, fmtMs, attitudeStatus, enumName } from './transforms.js';

export { attitudeStatus, enumName };

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

export const TRAIL_MAX = 4000;
const TRAIL_Z = 0.015;
const TRAIL_MIN_STEP_M = 0.001;
const FRUSTUM_DEPTH_M = 0.45;
const BOUNDARY_COLOR = 0x7ee787;
const OBSTACLE_COLOR = 0xff7b72;
const LANDMARK_BOX_COLOR = 0xffa657;
const LABEL_MIN_INTERVAL_MS = 1000;

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
// With opts.now a changed text is re-rasterized at most once per
// LABEL_MIN_INTERVAL_MS: callers pass the newest text every time, so the
// label catches up on a later call. opts.x/y/z place a new sprite.
function setLabel(parent, current, text, opts) {
    const s = String(text);
    if (current && current.userData.text === s) {
        return current;
    }
    const now = opts && typeof opts.now === 'number' ? opts.now : null;
    if (current && now !== null && now - current.userData.rasterAt < LABEL_MIN_INTERVAL_MS) {
        return current;
    }
    if (current) {
        parent.remove(current);
        disposeObject(current);
    }
    const sprite = textSprite(s, opts);
    sprite.userData.rasterAt = now === null ? 0 : now;
    if (opts) {
        sprite.position.set(opts.x || 0, opts.y || 0, opts.z || 0);
    }
    parent.add(sprite);
    return sprite;
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

// A planning rectangle as a wireframe with a faint fill, in its owner's
// frame: centered at (x, y), rotated by yaw, size_x along its own x.
function collisionBoxObject(box, color, info) {
    const group = new THREE.Group();
    group.position.set(box.x_m, box.y_m, 0);
    group.rotation.z = (box.yaw_deg || 0) * DEG;
    const hx = box.size_x_m / 2, hy = box.size_y_m / 2, z = 0.006;
    const outline = new THREE.LineLoop(
        new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(-hx, -hy, z), new THREE.Vector3(hx, -hy, z),
            new THREE.Vector3(hx, hy, z), new THREE.Vector3(-hx, hy, z)]),
        new THREE.LineBasicMaterial({ color }));
    group.add(outline);
    const fill = new THREE.Mesh(new THREE.PlaneGeometry(box.size_x_m, box.size_y_m),
        new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.14, depthWrite: false, side: THREE.DoubleSide }));
    fill.position.z = z - 0.001;
    group.add(fill);
    group.userData.info = info;
    return group;
}

function boxInfo(owner, frame, box) {
    return `${owner} collision box (${frame} frame, planning)\n` +
        `${fmt(box.size_x_m, 4)} x ${fmt(box.size_y_m, 4)} m at ${fmt(box.x_m)}, ${fmt(box.y_m)} yaw ${fmt(box.yaw_deg, 1)} deg\n` +
        `${box.note || ''}`;
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
        this.cameraMounts = new Map(); // camera id -> group
        this.highlighted = null;
        this.landmarkBoxes = new Map(); // landmark id -> nominal collision box
        this.planningGroup = null;      // boundary and fixed obstacles
        this.showPlanning = true;
        this.path = null;
        this.pathKey = '';
        this.pathInfo = null;           // {command_id, points} of the drawn path
        this.robotBodyKey = '';
        this.robotState = null;         // newest exact state, for the tooltip
        this.robotStale = false;
        this.robotPlaced = true;
        this.robotLabelText = '';
        this.tiltRoll = 0;
        this.tiltPitch = 0;
        this.drawnHeading = NaN;
        this.desiredHeadingDeg = NaN;
        this.mode = 'live';
        this.needsRender = true;
        this.renders = 0;
        this.lastRenderAt = -Infinity;
        this.dragging = false;
        this.pointerPending = false;
        this.pointerX = 0;
        this.pointerY = 0;
        this.tooltipText = '';
        this.tooltipW = 0;
        this.tooltipH = 0;
        this.followDelta = new THREE.Vector3();

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
        this.controls.addEventListener('change', () => {
            this.needsRender = true;
        });
        // no picking while the view is dragged: the raycast is the costly part
        this.controls.addEventListener('start', () => {
            this.dragging = true;
            this.hideTooltip();
        });
        this.controls.addEventListener('end', () => {
            this.dragging = false;
        });

        this.scene.add(new THREE.HemisphereLight(0xffffff, 0x3a3f4a, 1.1));
        const sun = new THREE.DirectionalLight(0xffffff, 1.4);
        sun.position.set(2, -3, 5);
        this.scene.add(sun);

        this.dynamic = new THREE.Group();
        this.scene.add(this.dynamic);
        this.buildRobot({ length_m: 0.45, width_m: 0.45, height_m: 0.3, origin_x_m: 0, origin_y_m: 0 });
        this.buildTrail();
        this.buildOverlays();

        this.raycaster = new THREE.Raycaster();
        this.pointer = new THREE.Vector2();
        this.tooltip = container.querySelector('#tooltip');
        // the move only records where the pointer is; the pick runs once
        // per animation frame at most (render())
        this.renderer.domElement.addEventListener('pointermove', (e) => {
            this.pointerX = e.clientX;
            this.pointerY = e.clientY;
            this.pointerPending = true;
        });
        this.renderer.domElement.addEventListener('pointerleave', () => {
            this.pointerPending = false;
            this.hideTooltip();
        });

        this.resize();
        // resize on the next frame: sizing the canvas inside the observer
        // callback would re-trigger it (undelivered-notifications error)
        const later = () => requestAnimationFrame(() => this.resize());
        if (typeof ResizeObserver !== 'undefined') {
            new ResizeObserver(later).observe(container);
        } else {
            window.addEventListener('resize', later);
        }
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
        this.rect = this.renderer.domElement.getBoundingClientRect();
        this.needsRender = true;
    }

    // One animation frame: trail upload, follow, damping, a pending pick,
    // then a render only when something changed. True when it rendered.
    render() {
        if (!this.available) {
            return false;
        }
        this.flushTrail();
        if (this.followRobot && this.robot.visible) {
            const d = this.followDelta.copy(this.robot.position).sub(this.controls.target);
            if (d.lengthSq() > 1e-12) {
                this.controls.target.add(d);
                this.camera.position.add(d);
                this.needsRender = true;
            }
        }
        if (this.controls.update()) {
            this.needsRender = true;
        }
        if (this.pointerPending && !this.dragging) {
            this.pointerPending = false;
            this.pick();
        }
        // an idle view still redraws once a second, so a screenshot or a
        // compositor that dropped the buffer never finds an empty canvas
        const now = performance.now();
        if (!this.needsRender && now - this.lastRenderAt < 1000) {
            return false;
        }
        this.needsRender = false;
        this.lastRenderAt = now;
        this.renderer.render(this.scene, this.camera);
        this.renders += 1;
        return true;
    }

    // --- views ---

    resetView() {
        if (!this.available) {
            return;
        }
        this.followRobot = false;
        const s = this.fieldSpan;
        this.controls.target.copy(this.fieldCenter);
        this.camera.position.set(this.fieldCenter.x, this.fieldCenter.y - 0.95 * s, 0.75 * s);
        this.controls.update();
        this.needsRender = true;
    }

    topDown() {
        if (!this.available) {
            return;
        }
        this.followRobot = false;
        const s = this.fieldSpan;
        this.controls.target.copy(this.fieldCenter);
        // a hair off the vertical keeps OrbitControls' spherical math regular
        this.camera.position.set(this.fieldCenter.x, this.fieldCenter.y - 0.001, 1.25 * s);
        this.controls.update();
        this.needsRender = true;
    }

    setFollow(on) {
        this.followRobot = on;
        this.needsRender = true;
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
        this.landmarkBoxes.clear();
        this.pickables = this.pickables.filter((p) => p === this.robot || p === this.dest || p === this.followTarget || p === this.path);
        this.families = hello.tag_families || {};
        this.fieldGroup = new THREE.Group();
        this.scene.add(this.fieldGroup);
        this.planningGroup = new THREE.Group();
        this.planningGroup.visible = this.showPlanning;
        this.fieldGroup.add(this.planningGroup);

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
                body.label = textSprite(shortId(lm.id) + (body.known ? ' (nominal)' : ' (nominal, no visual)'), { height_m: 0.07, bg: 'rgba(0,0,0,0.45)', color: '#d8dde6' });
                body.label.position.set(0, 0, body.height + 0.09);
                body.group.add(body.label);
                if (lm.collision_box) {
                    const box = collisionBoxObject(lm.collision_box, LANDMARK_BOX_COLOR,
                        boxInfo(`${lm.id} (wire id ${lm.wire_id}, nominal)`, 'landmark', lm.collision_box));
                    box.visible = this.showPlanning;
                    body.group.add(box);
                    this.landmarkBoxes.set(lm.id, box);
                }
                this.fieldGroup.add(body.group);
                this.nominal.set(lm.id, body);
                this.pickables.push(body.group);
            }
            this.buildPlanning(field);
            this.buildWallLabels(field);
        }
        this.fieldCenter.set(cx, cy, 0);
        this.fieldSpan = span;
        this.buildAxes();
        this.resetView();
    }

    // Boundary and fixed obstacles; landmark boxes ride on the landmark bodies.
    buildPlanning(field) {
        const b = field.boundary;
        if (b) {
            const z = 0.008;
            const loop = new THREE.LineLoop(
                new THREE.BufferGeometry().setFromPoints([
                    new THREE.Vector3(b.min_x_m, b.min_y_m, z), new THREE.Vector3(b.max_x_m, b.min_y_m, z),
                    new THREE.Vector3(b.max_x_m, b.max_y_m, z), new THREE.Vector3(b.min_x_m, b.max_y_m, z)]),
                new THREE.LineBasicMaterial({ color: BOUNDARY_COLOR }));
            this.planningGroup.add(loop);
        }
        for (const o of field.obstacles || []) {
            if (!o.collision_box) {
                continue;
            }
            const g = new THREE.Group();
            g.position.set(o.pose.x_m, o.pose.y_m, 0);
            g.rotation.z = o.pose.heading_deg * DEG;
            const box = collisionBoxObject(o.collision_box, OBSTACLE_COLOR,
                boxInfo(`${o.id} (wire id ${o.wire_id}, fixed)`, 'obstacle', o.collision_box));
            g.add(box);
            this.planningGroup.add(g);
            this.pickables.push(box);
        }
    }

    // Boundary, collision boxes and the reported path on or off.
    setPlanningVisible(on) {
        if (!this.available) {
            return;
        }
        this.showPlanning = on;
        if (this.planningGroup) {
            this.planningGroup.visible = on;
        }
        for (const box of this.landmarkBoxes.values()) {
            box.visible = on && !box.userData.observedElsewhere;
        }
        for (const e of this.estimates.values()) {
            if (e.box) {
                e.box.visible = on;
            }
        }
        if (this.path) {
            this.path.visible = on;
        }
        this.needsRender = true;
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
        this.robotMaterial = box.material;
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
        this.robotLabelText = 'robot';
        this.robotLabel = textSprite('robot', { height_m: 0.07 });
        this.robotLabel.position.set(0, 0, body.height_m + 0.1);
        robot.add(this.robotLabel);
        robot.userData.infoFn = () => this.robotInfo();
        this.robot = robot;
        this.robotBody = body;
        this.drawnHeading = NaN;
        this.dynamic.add(robot);
        this.cameraMounts.clear();
        this.pickables.push(robot);
        this.robotLook(this.robotPlaced, this.robotStale);
        this.needsRender = true;
    }

    // The running profile's footprint when one is applied, else RobotBody.
    robotBodyFor(diag, hello) {
        const rb = (hello && hello.robot_body) || { length_m: 0.45, width_m: 0.45, height_m: 0.3, origin_x_m: 0, origin_y_m: 0 };
        const link = diag ? diag.brain_link : null;
        const running = link && link.profile ? link.profile.running : null;
        if (running && running.footprint) {
            const f = running.footprint;
            return {
                length_m: f.front_m + f.back_m, width_m: f.left_m + f.right_m, height_m: rb.height_m || 0.3,
                origin_x_m: (f.front_m - f.back_m) / 2, origin_y_m: (f.left_m - f.right_m) / 2,
                source: `profile ${running.id} footprint`,
            };
        }
        return { ...rb, source: 'RobotBody (display only)' };
    }

    robotLook(placed, stale) {
        this.robotMaterial.color.setHex(placed ? 0xff9f43 : 0x8a8f98);
        // a stale pose is drawn faded so it cannot pass for a live one
        this.robotMaterial.opacity = stale ? 0.25 : (placed ? 0.85 : 0.45);
        this.needsRender = true;
    }

    // The newest exact state of the robot (not the smoothed pose): what the
    // label, colour, tilt and tooltip say. stale is the page's staleness
    // verdict (no state for a while, or the pose itself too old).
    applyRobotState(robot, stale) {
        if (!this.available) {
            return;
        }
        this.robotState = robot;
        if (!robot || !robot.valid) {
            if (this.robot.visible) {
                this.robot.visible = false;
                this.needsRender = true;
            }
            return;
        }
        if (!this.robot.visible) {
            this.robot.visible = true;
            this.needsRender = true;
        }
        const placed = !!robot.initialized;
        if (placed !== this.robotPlaced || stale !== this.robotStale) {
            this.robotPlaced = placed;
            this.robotStale = stale;
            this.robotLook(placed, stale);
        }
        const status = attitudeStatus(robot.attitude);
        const tilt = status === 'measured';
        const roll = tilt ? robot.attitude.roll_deg * DEG : 0;
        const pitch = tilt ? robot.attitude.pitch_deg * DEG : 0;
        if (roll !== this.tiltRoll || pitch !== this.tiltPitch) {
            this.tiltRoll = roll;
            this.tiltPitch = pitch;
            this.drawnHeading = NaN; // re-applies the rotation on the next placeRobot
        }
        let text;
        if (stale) {
            text = 'robot (pose stale)';
        } else if (!placed) {
            text = 'robot (not placed, odometry)';
        } else if (tilt) {
            text = 'robot';
        } else if (status === 'assumed_level') {
            text = 'robot (level assumed)';
        } else if (status === 'stale') {
            text = 'robot (attitude stale)';
        } else {
            text = 'robot (no attitude)';
        }
        if (text !== this.robotLabelText) {
            this.robotLabelText = text;
            this.robotLabel = setLabel(this.robot, this.robotLabel, text,
                { height_m: 0.07, z: this.robotBody.height_m + 0.1 });
            this.needsRender = true;
        }
    }

    // Draws the robot at a planar pose (smoothed in live, recorded in replay).
    placeRobot(pose) {
        if (!this.available || !pose) {
            return;
        }
        const r = this.robot;
        const h = pose.heading_deg * DEG;
        if (r.position.x !== pose.x_m || r.position.y !== pose.y_m || this.drawnHeading !== h) {
            r.position.set(pose.x_m, pose.y_m, 0);
            // R = Rz(yaw) Ry(pitch) Rx(roll), the document convention
            r.rotation.set(this.tiltRoll, this.tiltPitch, h, 'ZYX');
            this.drawnHeading = h;
            this.needsRender = true;
        }
        if (this.desired.visible) {
            this.desired.position.set(pose.x_m, pose.y_m, 0);
        }
    }

    robotInfo() {
        const robot = this.robotState;
        if (!robot || !robot.valid) {
            return 'robot: no valid pose';
        }
        const f = robot.field || {};
        const placed = !!robot.initialized;
        const att = robot.attitude || {};
        const status = attitudeStatus(att);
        const attText = status === 'measured'
            ? `roll ${fmt(att.roll_deg, 1)} pitch ${fmt(att.pitch_deg, 1)} deg (${att.source}, age ${fmtMs(att.age_ms)})`
            : `attitude ${status.replace('_', ' ')}`;
        const placedBy = robot.placement_origin === 'command' ? 'Brain' : robot.placement_origin;
        return (this.mode === 'replay' ? 'REPLAY: recorded state\n' : '') +
            (placed ? `robot origin, placed by ${placedBy}` : 'robot origin, NOT PLACED: odometry pose, not a field position') +
            `\nx ${fmt(f.x_m)} m  y ${fmt(f.y_m)} m  heading ${fmt(f.heading_deg, 1)} deg (exact, not smoothed)\n` +
            `pose age ${fmtMs(robot.age_ms)} at publish  confidence ${fmt(robot.confidence, 2)}\n${attText}\n` +
            `outline: ${this.robotBody.source || 'RobotBody'}, ${fmt(this.robotBody.length_m, 3)} x ${fmt(this.robotBody.width_m, 3)} m` +
            (this.robotStale ? '\nSTALE: no fresh pose' : '');
    }

    // --- trail: a ring of odometry-frame points ---
    //
    // Each point is written twice, at slot i and i + TRAIL_MAX, so the
    // newest TRAIL_MAX points are always one contiguous draw range and a
    // new point uploads only its own bytes. The line object carries
    // field_from_odom, so a placement or correction re-expresses the whole
    // trail without touching the buffer; a new odometry epoch clears it.

    buildTrail() {
        this.trailBuf = new Float32Array(TRAIL_MAX * 2 * 3);
        this.trailT = new Float64Array(TRAIL_MAX);
        this.trailAttr = new THREE.BufferAttribute(this.trailBuf, 3).setUsage(THREE.DynamicDrawUsage);
        const geom = new THREE.BufferGeometry().setAttribute('position', this.trailAttr);
        geom.setDrawRange(0, 0);
        this.trail = new THREE.Line(geom, new THREE.LineBasicMaterial({ color: 0xffd166 }));
        this.trail.frustumCulled = false;
        this.trail.visible = false;
        this.dynamic.add(this.trail);
        this.trailHead = 0;
        this.trailCount = 0;
        this.trailLastT = -Infinity;
        this.trailDirtyLo = TRAIL_MAX;
        this.trailDirtyHi = -1;
        this.trailWrites = 0;
        this.trailUploadedBytes = 0;
        this.trailEpoch = null;
    }

    clearTrail() {
        if (!this.available) {
            return;
        }
        this.trailHead = 0;
        this.trailCount = 0;
        this.trailLastT = -Infinity;
        this.trailDirtyLo = TRAIL_MAX;
        this.trailDirtyHi = -1;
        this.trailWrites = 0;
        if (this.trail) {
            this.trail.geometry.setDrawRange(0, 0);
        }
        this.needsRender = true;
    }

    // Appends one point at time t (ms); near-duplicates only move the time.
    trailPush(x, y, t) {
        if (!this.available) {
            return;
        }
        const N = TRAIL_MAX;
        if (this.trailCount) {
            const last = this.trailHead === 0 ? N - 1 : this.trailHead - 1;
            const dx = x - this.trailBuf[last * 3];
            const dy = y - this.trailBuf[last * 3 + 1];
            if (dx * dx + dy * dy < TRAIL_MIN_STEP_M * TRAIL_MIN_STEP_M) {
                this.trailT[last] = t;
                this.trailLastT = t;
                return;
            }
        }
        const i = this.trailHead;
        const a = i * 3;
        const b = (i + N) * 3;
        this.trailBuf[a] = this.trailBuf[b] = x;
        this.trailBuf[a + 1] = this.trailBuf[b + 1] = y;
        this.trailBuf[a + 2] = this.trailBuf[b + 2] = TRAIL_Z;
        this.trailT[i] = t;
        this.trailLastT = t;
        this.trailHead = i + 1 === N ? 0 : i + 1;
        if (this.trailCount < N) {
            this.trailCount += 1;
        }
        if (i < this.trailDirtyLo) {
            this.trailDirtyLo = i;
        }
        if (i > this.trailDirtyHi) {
            this.trailDirtyHi = i;
        }
        this.trailWrites += 1;
    }

    // Uploads only the slots written since the last frame (both copies).
    flushTrail() {
        if (this.trailWrites === 0) {
            return;
        }
        const N = TRAIL_MAX;
        const attr = this.trailAttr;
        attr.clearUpdateRanges();
        const lo = this.trailDirtyLo;
        const hi = this.trailDirtyHi;
        const n = hi - lo + 1;
        if (this.trailWrites >= N || n > N / 2) {
            this.trailUploadedBytes += this.trailBuf.byteLength;  // whole buffer
        } else {
            attr.addUpdateRange(lo * 3, n * 3);
            attr.addUpdateRange((lo + N) * 3, n * 3);
            this.trailUploadedBytes += n * 2 * 12;
        }
        attr.needsUpdate = true;
        const start = this.trailHead - this.trailCount < 0 ? this.trailHead - this.trailCount + N : this.trailHead - this.trailCount;
        this.trail.geometry.setDrawRange(start, this.trailCount);
        this.trailDirtyLo = N;
        this.trailDirtyHi = -1;
        this.trailWrites = 0;
        this.needsRender = true;
    }

    // Points newer than t, oldest first, as [x, y, t] (history merges only).
    trailSince(t) {
        const out = [];
        const N = TRAIL_MAX;
        for (let k = 0; k < this.trailCount; ++k) {
            let i = this.trailHead - this.trailCount + k;
            if (i < 0) {
                i += N;
            }
            if (this.trailT[i] > t) {
                out.push([this.trailBuf[i * 3], this.trailBuf[i * 3 + 1], this.trailT[i]]);
            }
        }
        return out;
    }

    // The frame the trail points are in: field_from_odom in live, identity
    // (field coordinates) in replay.
    setTrailFrame(anchor) {
        if (!this.available) {
            return;
        }
        const x = anchor ? anchor.x_m : 0;
        const y = anchor ? anchor.y_m : 0;
        const h = anchor ? anchor.heading_deg * DEG : 0;
        const t = this.trail;
        if (t.position.x !== x || t.position.y !== y || t.rotation.z !== h) {
            t.position.set(x, y, 0);
            t.rotation.z = h;
            this.needsRender = true;
        }
        if (!t.visible) {
            t.visible = true;
            this.needsRender = true;
        }
    }

    // Live: a state's exact odometry pose joins the trail when it is a new
    // valid measurement of the trail's epoch (the server's history rule).
    trailFromState(robot) {
        if (!this.available) {
            return;
        }
        if (!robot || !robot.valid || typeof robot.measured_at_host_ms !== 'number') {
            return;
        }
        if (this.trailEpoch !== robot.odometry_epoch) {
            this.clearTrail();
            this.trailEpoch = robot.odometry_epoch;
        }
        if (robot.measured_at_host_ms > this.trailLastT && robot.odom) {
            this.trailPush(robot.odom.x_m, robot.odom.y_m, robot.measured_at_host_ms);
        }
        if (robot.field_from_odom) {
            this.setTrailFrame(robot.field_from_odom);
        }
    }

    // A history batch (oldest first, odometry frame). replace: rebuild from
    // it and keep what arrived after it; merge: append what is newer.
    trailFromHistory(entries, replace, currentEpoch) {
        if (!this.available) {
            return;
        }
        let epoch = currentEpoch;
        if (epoch === null || epoch === undefined) {
            epoch = entries.length ? entries[entries.length - 1].epoch : this.trailEpoch;
        }
        if (this.trailEpoch !== epoch) {
            this.clearTrail();
            this.trailEpoch = epoch;
        }
        let keep = null;
        if (replace) {
            let lastT = -Infinity;
            for (const e of entries) {
                if (e.epoch === epoch && e.host_ms > lastT) {
                    lastT = e.host_ms;
                }
            }
            keep = this.trailSince(lastT);
            this.clearTrail();
            this.trailEpoch = epoch;
        }
        for (const e of entries) {
            if (e.epoch !== epoch || !(e.host_ms > this.trailLastT)) {
                continue;
            }
            this.trailPush(e.x_m, e.y_m, e.host_ms);
        }
        if (keep) {
            for (const [x, y, t] of keep) {
                if (t > this.trailLastT) {
                    this.trailPush(x, y, t);
                }
            }
        }
    }

    // --- per diag message ---

    applyDiag(diag, hello, now) {
        if (!this.available || !diag) {
            return;
        }
        const body = this.robotBodyFor(diag, hello);
        const bodyKey = `${body.length_m},${body.width_m},${body.height_m},${body.origin_x_m},${body.origin_y_m},${body.source}`;
        if (bodyKey !== this.robotBodyKey) {
            const wasVisible = this.robot.visible;
            this.robotBodyKey = bodyKey;
            this.buildRobot(body);
            this.robot.visible = wasVisible;
            this.robotLabelText = '';
            this.applyRobotState(this.robotState, this.robotStale);
        }
        this.updateCameraMounts(diag, hello);
        if (this.mode === 'live') {
            this.updateEstimates(diag, now);
            this.updatePath(diag.brain_link ? diag.brain_link.path : null);
        }
    }

    // The Brain's latest PATH_REPORT, field frame, inspection only: a line
    // and one Points draw for the vertices, not a mesh per vertex.
    updatePath(p) {
        if (!this.available) {
            return;
        }
        const key = p ? `${p.session}/${p.command_id}/${p.received_host_ms}/${p.mode}/${(p.points || []).length}` : '';
        if (key === this.pathKey) {
            return;
        }
        this.pathKey = key;
        this.clearPath();
        if (!p || !Array.isArray(p.points) || p.points.length === 0) {
            return;
        }
        const color = p.mode === 'avoiding' ? 0xff66cc : 0x66e0ff;
        const z = 0.03;
        const group = new THREE.Group();
        const pos = new Float32Array(p.points.length * 3);
        p.points.forEach((q, i) => {
            pos[i * 3] = q.x_m;
            pos[i * 3 + 1] = q.y_m;
            pos[i * 3 + 2] = z;
        });
        const geom = new THREE.BufferGeometry().setAttribute('position', new THREE.BufferAttribute(pos, 3));
        const line = new THREE.Line(geom, new THREE.LineBasicMaterial({ color }));
        line.frustumCulled = false;
        group.add(line);
        const dots = new THREE.Points(geom, new THREE.PointsMaterial({ color, size: 7, sizeAttenuation: false }));
        dots.frustumCulled = false;
        group.add(dots);
        const last = p.points[p.points.length - 1];
        const label = textSprite(`path ${p.mode} #${p.command_id}`, { height_m: 0.06, color: '#ffd6f2' });
        label.position.set(last.x_m, last.y_m, 0.12);
        group.add(label);
        group.userData.info = (this.mode === 'replay' ? 'REPLAY: recorded path\n' : '') +
            'planned path reported by the Brain (inspection only)\n' +
            `mode ${p.mode}, command ${p.command_id}, session ${p.session}\n` +
            `${p.points.length} points` + (typeof p.age_ms === 'number' ? `, reported ${fmtMs(p.age_ms)} before this message` : '');
        group.visible = this.showPlanning;
        this.path = group;
        this.pathInfo = { command_id: p.command_id, points: p.points };
        this.dynamic.add(group);
        this.pickables.push(group);
        this.needsRender = true;
    }

    clearPath() {
        if (!this.available) {
            return;
        }
        this.pathInfo = null;
        if (!this.path) {
            return;
        }
        this.dynamic.remove(this.path);
        disposeObject(this.path);
        this.pickables = this.pickables.filter((q) => q !== this.path);
        this.path = null;
        this.needsRender = true;
    }

    updateCameraMounts(diag, hello) {
        const seen = new Set();
        const frames = diag.detection_frames || [];
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
            const K = s.K;
            const T = s.T;
            const sig = `${s.mounted}|${K ? [K.width_px, K.height_px, K.fx_px, K.fy_px, K.cx_px, K.cy_px].join(',') : ''}|` +
                `${T ? [T.x_m, T.y_m, T.z_m, T.roll_deg, T.pitch_deg, T.yaw_deg].join(',') : ''}`;
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
            if (T) {
                g.matrixAutoUpdate = false;
                g.matrix.copy(matrixFromTransform(T));
            }
            const calibrated = !!K;
            const color = calibrated && s.mounted ? 0x4fc3f7 : 0xffb347;
            if (calibrated) {
                g.add(frustumLines(K, color));
            }
            const marker = new THREE.Mesh(new THREE.SphereGeometry(0.018, 12, 8), new THREE.MeshBasicMaterial({ color }));
            g.add(marker);
            g.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.12, color, 0.03, 0.02));
            const label = textSprite(`${s.id}${calibrated ? '' : ' (no intrinsics)'}${s.mounted ? '' : ' (unmounted)'}`,
                { height_m: 0.05, color: '#bfe9ff' });
            label.position.set(0, 0, 0.06);
            g.add(label);
            g.userData.info = `camera ${s.id}\n${s.mounted && T ? 'T_robot_camera ' + fmt(T.x_m) + ', ' + fmt(T.y_m) + ', ' + fmt(T.z_m) + ' m rpy ' + fmt(T.roll_deg, 1) + ', ' + fmt(T.pitch_deg, 1) + ', ' + fmt(T.yaw_deg, 1) + ' deg' : 'not mounted: drawn at the robot origin'}\n` +
                (calibrated ? 'frustum from intrinsics' : 'no intrinsics: marker only');
            this.robot.add(g);
            this.cameraMounts.set(s.id, g);
            this.needsRender = true;
        }
        for (const [id, g] of this.cameraMounts) {
            if (!seen.has(id)) {
                this.robot.remove(g);
                disposeObject(g);
                this.cameraMounts.delete(id);
                this.needsRender = true;
            }
        }
    }

    updateEstimates(diag, now) {
        const seen = new Set();
        for (const o of diag.field_objects || []) {
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
                let box = null;
                if (decl.collision_box) {
                    box = collisionBoxObject(decl.collision_box, LANDMARK_BOX_COLOR,
                        boxInfo(`${o.id} (wire id ${decl.wire_id}, estimate)`, 'landmark', decl.collision_box));
                    box.visible = this.showPlanning;
                    body.group.add(box);
                }
                e = { ...body, line, label: null, id: o.id, box };
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
            // no age in the 3D label (the landmark table has it): the label
            // re-rasterizes on a real change, at most once a second
            const text = `${shortId(o.id)} est.\n` +
                (nominal ? `d ${fmt(o.displacement_m, 3)} m  dh ${fmt(o.heading_error_deg, 1)} deg` : 'no nominal') +
                (o.valid ? '' : '\n(invalid)');
            e.label = setLabel(e.group, e.label, text, {
                height_m: 0.06, bg: 'rgba(40,30,0,0.75)', color: '#ffe9a8', z: e.height + 0.22, now,
            });
            e.state = o;
            if (!e.group.userData.infoFn) {
                e.group.userData.infoFn = () => estimateInfo(e.state);
            }
            this.needsRender = true;
        }
        for (const [id, e] of this.estimates) {
            if (!seen.has(id)) {
                this.dynamic.remove(e.group, e.line);
                disposeObject(e.group);
                disposeObject(e.line);
                this.estimates.delete(id);
                this.pickables = this.pickables.filter((p) => p !== e.group);
                this.needsRender = true;
            }
        }
        // a landmark box is drawn once: at the estimate when observed, else nominal
        for (const [id, box] of this.landmarkBoxes) {
            box.userData.observedElsewhere = seen.has(id);
            box.visible = this.showPlanning && !seen.has(id);
        }
    }

    clearEstimates() {
        if (!this.available) {
            return;
        }
        const groups = new Set();
        for (const e of this.estimates.values()) {
            groups.add(e.group);
            this.dynamic.remove(e.group, e.line);
            disposeObject(e.group);
            disposeObject(e.line);
        }
        this.estimates.clear();
        this.pickables = this.pickables.filter((p) => !groups.has(p));
        for (const box of this.landmarkBoxes.values()) {
            box.userData.observedElsewhere = false;
            box.visible = this.showPlanning;
        }
        this.needsRender = true;
    }

    // --- telemetry overlays: destination, desired heading, follower target ---

    buildOverlays() {
        this.overlay = new THREE.Group();
        this.dynamic.add(this.overlay);
        const color = 0x7ee787;
        this.dest = new THREE.Group();
        this.destMaterial = new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide, transparent: true, opacity: 0.9, depthWrite: false });
        const ring = new THREE.Mesh(new THREE.RingGeometry(0.07, 0.095, 40), this.destMaterial);
        ring.position.z = 0.02;
        this.dest.add(ring);
        this.dest.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0.02), 0.28, color, 0.08, 0.05));
        this.destLabel = null;
        this.destLabelText = '';
        this.dest.visible = false;
        this.dest.userData.infoFn = () => this.telemetryInfo();
        this.overlay.add(this.dest);
        this.pickables.push(this.dest);
        // desired heading (the destination heading) drawn at the robot
        this.desired = new THREE.Group();
        this.desiredArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0.35), 0.4, color, 0.08, 0.05);
        this.desired.add(this.desiredArrow);
        this.desired.visible = false;
        this.overlay.add(this.desired);
        // the reported path vertex the follower's current segment ends at
        this.followTarget = new THREE.Mesh(new THREE.OctahedronGeometry(0.035),
            new THREE.MeshBasicMaterial({ color: 0xffffff }));
        this.followTarget.visible = false;
        this.followTarget.userData.info = '';
        this.overlay.add(this.followTarget);
        this.pickables.push(this.followTarget);
        this.telemetryState = null;
    }

    hideOverlays() {
        if (!this.available) {
            return;
        }
        if (this.dest.visible || this.desired.visible || this.followTarget.visible) {
            this.dest.visible = false;
            this.desired.visible = false;
            this.followTarget.visible = false;
            this.needsRender = true;
        }
    }

    // tel: a telemetry message (or a replay row shaped like one); fresh:
    // whether it is recent enough to draw. Nothing is guessed: the target
    // carries no validity bit yet, so (0, 0, 0) can mean "no destination"; it
    // is drawn only while the command runs or settles (or when a future
    // target-valid flag, bit 3, says so). The follower target is drawn only
    // when the reported path is the command's own and has one vertex per
    // follower segment boundary.
    applyTelemetry(tel, fresh) {
        if (!this.available) {
            return;
        }
        this.telemetryState = tel;
        const m = tel ? tel.motion : null;
        const stateName = m ? enumName(m.state, m.state_name) : '';
        const targetFlag = tel && typeof tel.flags === 'number' && (tel.flags & 8) !== 0;
        const active = stateName === 'running' || stateName === 'settling' || m && (m.state === 2 || m.state === 3);
        if (!m || !fresh || !m.target || !(active || targetFlag)) {
            this.hideOverlays();
            return;
        }
        const terminal = !active;
        const t = m.target;
        this.dest.position.set(t.x_m, t.y_m, 0);
        this.dest.rotation.z = t.heading_deg * DEG;
        this.destMaterial.opacity = terminal ? 0.35 : 0.9;
        this.dest.visible = true;
        const text = `destination #${m.command_id} ${stateName}`;
        if (text !== this.destLabelText) {
            this.destLabelText = text;
            this.destLabel = setLabel(this.dest, this.destLabel, text, { height_m: 0.055, color: '#caffd8', z: 0.16 });
        }
        this.desired.rotation.z = t.heading_deg * DEG;
        this.desired.visible = !terminal && this.robot.visible;
        if (this.desired.visible) {
            this.desired.position.copy(this.robot.position);
        }
        const path = this.pathInfo;
        const n = path ? path.points.length : 0;
        const ok = path && path.command_id === m.command_id && typeof m.segment === 'number' &&
            typeof m.segment_count === 'number' && m.segment_count + 1 === n && m.segment < m.segment_count;
        if (ok && !terminal) {
            const q = path.points[m.segment + 1];
            this.followTarget.position.set(q.x_m, q.y_m, 0.06);
            this.followTarget.userData.info = `follower target: end of segment ${m.segment + 1} of ${m.segment_count}\n` +
                `(${fmt(q.x_m)}, ${fmt(q.y_m)}) m, path #${path.command_id}`;
            this.followTarget.visible = true;
        } else {
            this.followTarget.visible = false;
        }
        this.needsRender = true;
    }

    telemetryInfo() {
        const tel = this.telemetryState;
        const m = tel && tel.motion;
        if (!m) {
            return 'no motion telemetry';
        }
        const c = m.cmd || {};
        const t = m.target || {};
        return (this.mode === 'replay' ? 'REPLAY: recorded telemetry\n' : '') +
            `destination of command #${m.command_id} (Brain telemetry, field frame)\n` +
            `x ${fmt(t.x_m)} m  y ${fmt(t.y_m)} m  heading ${fmt(t.heading_deg, 1)} deg\n` +
            `state ${enumName(m.state, m.state_name)}, reason ${enumName(m.reason, m.reason_name)}, mode ${enumName(m.mode, m.mode_name)}\n` +
            `segment ${m.segment} of ${m.segment_count}\n` +
            `commanded vx ${fmt(c.vx_m_s)} vy ${fmt(c.vy_m_s)} m/s, omega ${fmt(c.omega_deg_s, 1)} deg/s (body)\n` +
            `distance error ${fmt(m.distance_error_m)} m, cross-track ${fmt(m.cross_track_m)} m, heading error ${fmt(m.heading_error_deg, 1)} deg\n` +
            `drive fault ${enumName(m.drive_fault, m.drive_fault_name)}`;
    }

    // --- sessions, modes ---

    // Everything bound to a session: estimates, trail, path, overlays.
    clearSession() {
        if (!this.available) {
            return;
        }
        this.clearEstimates();
        this.clearTrail();
        this.trailEpoch = null;
        this.trail.visible = false;
        this.clearPath();
        this.pathKey = '';
        this.hideOverlays();
        this.robot.visible = false;
        this.robotState = null;
        this.needsRender = true;
    }

    // 'live' or 'replay'. Switching clears what belongs to the other mode;
    // static field geometry and camera mounts stay.
    setMode(mode) {
        if (mode === this.mode) {
            return;
        }
        this.mode = mode;
        this.clearSession();
    }

    setHighlight(id) {
        if (!this.available) {
            return;
        }
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
        this.needsRender = true;
    }

    // --- hover ---

    pick() {
        const rect = this.rect;
        if (!rect || !rect.width) {
            return;
        }
        this.pointer.x = ((this.pointerX - rect.left) / rect.width) * 2 - 1;
        this.pointer.y = -((this.pointerY - rect.top) / rect.height) * 2 + 1;
        this.raycaster.setFromCamera(this.pointer, this.camera);
        const hits = this.raycaster.intersectObjects(this.pickables, true);
        let info = null;
        const shown = (o) => {
            for (let p = o; p; p = p.parent) {
                if (!p.visible) {
                    return false;
                }
            }
            return true;
        };
        for (const h of hits) {
            if (!shown(h.object)) {
                continue;
            }
            let o = h.object;
            while (o && !o.userData.info && !o.userData.infoFn) {
                o = o.parent;
            }
            if (o) {
                info = o.userData.infoFn ? o.userData.infoFn() : o.userData.info;
                break;
            }
        }
        if (!info) {
            this.hideTooltip();
            return;
        }
        const tip = this.tooltip;
        if (info !== this.tooltipText) {
            this.tooltipText = info;
            tip.textContent = info;
            tip.style.display = 'block';
            // measured once per text, not on every move
            this.tooltipW = tip.offsetWidth;
            this.tooltipH = tip.offsetHeight;
        } else {
            tip.style.display = 'block';
        }
        const x = this.pointerX - rect.left + 14;
        const y = this.pointerY - rect.top + 14;
        tip.style.left = Math.max(0, Math.min(x, rect.width - this.tooltipW - 8)) + 'px';
        tip.style.top = Math.max(0, Math.min(y, rect.height - this.tooltipH - 8)) + 'px';
    }

    hideTooltip() {
        if (this.tooltip && this.tooltipText !== '') {
            this.tooltipText = '';
            this.tooltip.style.display = 'none';
        }
    }
}

function estimateInfo(o) {
    if (!o) {
        return '';
    }
    const nominal = o.nominal;
    return `${o.id} (${o.source}${o.observed ? ', observed this cycle' : ''})\n` +
        `x ${fmt(o.pose.x_m)} m  y ${fmt(o.pose.y_m)} m  heading ${fmt(o.pose.heading_deg, 1)} deg\n` +
        (nominal ? `nominal ${fmt(nominal.x_m)}, ${fmt(nominal.y_m)}, ${fmt(nominal.heading_deg, 1)} deg\n` +
            `displacement ${fmt(o.displacement_m, 3)} m  heading error ${fmt(o.heading_error_deg, 2)} deg\n` : '') +
        `age ${fmtMs(o.age_ms)}  confidence ${fmt(o.confidence, 2)}  valid ${o.valid}\n` +
        `last ${o.last_source} / ${o.last_feature} seq ${o.last_source_sequence}\n` +
        `anchor rev ${o.anchor_revision}  odometry epoch ${o.odometry_epoch}`;
}
