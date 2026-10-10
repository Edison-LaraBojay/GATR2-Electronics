// scene.js
// The three.js view: field meshes (built once per field change), the robot
// box, the trail, the conventions overlay and the camera modes. Field
// rendering is adapted from the archived Pi viewer
// (pi/naviGATR/spectaGATR/field_scene.js at commit 10c568e).
//
// Scene axes are the field axes one to one (transforms.js fieldToScene):
// x right on the diagram, y up on the diagram, z up; Object3D.DEFAULT_UP is
// +Z. No mirroring, so the right-handed field stays right-handed on screen.

import * as THREE from 'three';
import { OrbitControls } from './vendor/three/OrbitControls.js';
import {
    DEG, FOLLOW_OFFSET, SCENE_UP, fieldToScene, fmt, orbitCamera, overheadCamera,
} from './transforms.js';

THREE.Object3D.DEFAULT_UP.set(SCENE_UP[0], SCENE_UP[1], SCENE_UP[2]);

const FOV_OVERHEAD_DEG = 20;
const FOV_3D_DEG = 50;
const BG_COLOR = 0x1b1f26;
const ROBOT_COLOR = 0xff9f43;
const FRONT_COLOR = 0xffffff;
const LEFT_COLOR = 0x2ecc71;
const AXIS_X_COLOR = 0xff4d4d;
const AXIS_Y_COLOR = 0x2ecc71;
const AXIS_Z_COLOR = 0x4d8dff;
const ARC_COLOR = 0x4fc3f7;
const TRAIL_COLOR = 0xffd166;
const DIM_OPACITY = 0.3;

export function disposeObject(root) {
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

// Text drawn on a 2D canvas as a sprite: OS font, no network. height_m is
// the world height of one line; center is the sprite anchor (0..1).
function textSprite(text, opts = {}) {
    const px = 28;
    const lineH = px + 6;
    const lines = String(text).split('\n');
    const canvas = document.createElement('canvas');
    const ctx = canvas.getContext('2d');
    const font = `${opts.bold ? 'bold ' : ''}${px}px system-ui, "Segoe UI", sans-serif`;
    ctx.font = font;
    const w = Math.ceil(Math.max(...lines.map((l) => ctx.measureText(l).width))) + 16;
    const h = lines.length * lineH + 8;
    canvas.width = w;
    canvas.height = h;
    ctx.font = font;
    ctx.fillStyle = opts.bg || 'rgba(0,0,0,0.6)';
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
    if (opts.center) {
        sprite.center.set(opts.center[0], opts.center[1]);
    }
    sprite.renderOrder = 10;
    return sprite;
}

function arrow(dir, origin, length, color, head = 0.08, headW = 0.05) {
    return new THREE.ArrowHelper(new THREE.Vector3(...dir), new THREE.Vector3(...origin), length, color, head, headW);
}

// --- field ---

function octagonGeometry(o) {
    // across-flats to circumradius; rotateZ(pi/8) puts flats on the axes
    const rb = o.base_across_flats_m / 2 / Math.cos(Math.PI / 8);
    const rt = o.top_across_flats_m / 2 / Math.cos(Math.PI / 8);
    const g = new THREE.CylinderGeometry(rt, rb, o.height_m, 8, 1, false);
    g.rotateX(Math.PI / 2);
    g.rotateZ(Math.PI / 8);
    g.translate(0, 0, o.height_m / 2);
    return g;
}

function buildObject(o) {
    const color = new THREE.Color(o.color || '#999999');
    const yaw = (o.yaw_deg || 0) * DEG;
    if (o.kind === 'box') {
        const mesh = new THREE.Mesh(
            new THREE.BoxGeometry(o.size_x_m, o.size_y_m, o.size_z_m),
            new THREE.MeshStandardMaterial({ color, roughness: 0.8 }));
        mesh.position.set(...fieldToScene(o.x_m, o.y_m, o.z_m));
        mesh.rotation.z = yaw;
        return mesh;
    }
    if (o.kind === 'tape') {
        const t = 0.003;
        const mesh = new THREE.Mesh(
            new THREE.BoxGeometry(o.size_x_m, o.size_y_m, t),
            new THREE.MeshStandardMaterial({ color, roughness: 0.8 }));
        mesh.position.set(...fieldToScene(o.x_m, o.y_m, t / 2));
        mesh.rotation.z = yaw;
        return mesh;
    }
    if (o.kind === 'octagon') {
        const g = new THREE.Group();
        const geom = octagonGeometry(o);
        g.add(new THREE.Mesh(geom, new THREE.MeshStandardMaterial({ color, roughness: 0.7 })));
        g.add(new THREE.LineSegments(new THREE.EdgesGeometry(geom, 25),
            new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.6 })));
        g.position.set(...fieldToScene(o.x_m, o.y_m, 0));
        g.rotation.z = yaw;
        return g;
    }
    return null;
}

// Everything static of one field definition (fields/README.md schema).
export function buildFieldGroup(field) {
    const group = new THREE.Group();
    const d = field.dimensions;
    const floor = new THREE.Mesh(
        new THREE.PlaneGeometry(d.inside_x_m, d.inside_y_m),
        new THREE.MeshStandardMaterial({ color: 0x4a5058, roughness: 0.95 }));
    floor.position.set(...fieldToScene(d.inside_x_m / 2, d.inside_y_m / 2, -0.001));
    group.add(floor);
    if (d.tile_m > 0) {
        const pts = [];
        for (let x = 0; x <= d.inside_x_m + 1e-6; x += d.tile_m) {
            pts.push(new THREE.Vector3(x, 0, 0.001), new THREE.Vector3(x, d.inside_y_m, 0.001));
        }
        for (let y = 0; y <= d.inside_y_m + 1e-6; y += d.tile_m) {
            pts.push(new THREE.Vector3(0, y, 0.001), new THREE.Vector3(d.inside_x_m, y, 0.001));
        }
        group.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(pts),
            new THREE.LineBasicMaterial({ color: 0x6b727c })));
    }
    const t = d.wall_thickness_m || 0.05;
    const h = d.wall_height_m || 0.29;
    const wallMat = new THREE.MeshStandardMaterial({ color: 0x8d949e, transparent: true, opacity: 0.45, roughness: 0.6 });
    const walls = [
        [d.inside_x_m + 2 * t, t, d.inside_x_m / 2, -t / 2],
        [d.inside_x_m + 2 * t, t, d.inside_x_m / 2, d.inside_y_m + t / 2],
        [t, d.inside_y_m, -t / 2, d.inside_y_m / 2],
        [t, d.inside_y_m, d.inside_x_m + t / 2, d.inside_y_m / 2],
    ];
    for (const [sx, sy, x, y] of walls) {
        const wall = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, h), wallMat);
        wall.position.set(x, y, h / 2);
        group.add(wall);
    }
    for (const o of field.objects || []) {
        const obj = buildObject(o);
        if (obj) {
            group.add(obj);
        }
    }
    const w = field.walls || {};
    const label = (spec, text, x, y, center) => {
        if (spec && spec.label) {
            const s = textSprite(`${spec.label} (${text})`, { height_m: 0.1, color: spec.color || '#d8dde6', center });
            s.position.set(x, y, h + 0.05);
            group.add(s);
        }
    };
    label(w.x_min, 'x = 0', -t - 0.04, d.inside_y_m / 2, [1, 0.5]);
    label(w.x_max, `x = ${fmt(d.inside_x_m, 2)}`, d.inside_x_m + t + 0.04, d.inside_y_m / 2, [0, 0.5]);
    label(w.y_min, 'y = 0', d.inside_x_m / 2, -t - 0.04, [0.5, 1]);
    label(w.y_max, `y = ${fmt(d.inside_y_m, 2)}`, d.inside_x_m / 2, d.inside_y_m + t + 0.04, [0.5, 0]);
    return group;
}

// Origin, field axes, heading zero and the counterclockwise-positive arc.
function buildConventions() {
    const g = new THREE.Group();
    const z = 0.02;
    const ring = new THREE.Mesh(new THREE.RingGeometry(0.035, 0.055, 32),
        new THREE.MeshBasicMaterial({ color: 0xffffff, side: THREE.DoubleSide }));
    ring.position.set(0, 0, z);
    g.add(ring);
    const o = textSprite('origin (0, 0)', { height_m: 0.07, center: [0, 1] });
    o.position.set(0.06, -0.07, z);
    g.add(o);
    const len = 0.6;
    g.add(arrow([1, 0, 0], [0, 0, z], len, AXIS_X_COLOR, 0.1, 0.06));
    g.add(arrow([0, 1, 0], [0, 0, z], len, AXIS_Y_COLOR, 0.1, 0.06));
    g.add(arrow([0, 0, 1], [0, 0, z], 0.35, AXIS_Z_COLOR, 0.08, 0.05));
    const lx = textSprite('+X  (heading 0)', { height_m: 0.075, color: '#ff9b9b', center: [0, 0.5] });
    lx.position.set(len + 0.04, 0.0, z);
    g.add(lx);
    const ly = textSprite('+Y  (heading +90)', { height_m: 0.075, color: '#8ef0b4', center: [0, 0] });
    ly.position.set(0.02, len + 0.04, z);
    g.add(ly);
    const lz = textSprite('+Z up', { height_m: 0.06, color: '#9fc0ff', center: [0, 0.5] });
    lz.position.set(0.03, 0.03, 0.4);
    g.add(lz);
    g.userData.only3d = [lz];
    // the arc runs counterclockwise from +X toward +Y, arrowhead at the end
    const r = 0.36, a0 = 8 * DEG, a1 = 78 * DEG;
    const pts = [];
    for (let i = 0; i <= 32; ++i) {
        const a = a0 + (a1 - a0) * i / 32;
        pts.push(new THREE.Vector3(r * Math.cos(a), r * Math.sin(a), z));
    }
    g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: ARC_COLOR })));
    const tip = new THREE.Vector3(r * Math.cos(a1), r * Math.sin(a1), z);
    const tangent = new THREE.Vector3(-Math.sin(a1), Math.cos(a1), 0);
    g.add(new THREE.ArrowHelper(tangent, tip.clone().addScaledVector(tangent, -0.06), 0.06, ARC_COLOR, 0.06, 0.04));
    const la = textSprite('+heading (CCW)', { height_m: 0.065, color: '#a8e4ff', center: [0, 0.5] });
    la.position.set(r * Math.cos(45 * DEG) + 0.04, r * Math.sin(45 * DEG) + 0.04, z);
    g.add(la);
    return g;
}

// --- robot ---

// The robot box about its pose origin (hello.robot), with its markers: white
// front face and front edge, green left face and left edge, red +x
// (forward), green +y (left) and blue +z (up) arrows. Returns the group;
// group.userData.materials lists what dims when the pose is not live.
export function buildRobot(dims) {
    const L = dims.front_m + dims.back_m;
    const W = dims.left_m + dims.right_m;
    const H = dims.height_m;
    const cx = (dims.front_m - dims.back_m) / 2;
    const cy = (dims.left_m - dims.right_m) / 2;
    const group = new THREE.Group();
    const mats = [];
    const std = (color) => {
        const m = new THREE.MeshStandardMaterial({ color, roughness: 0.6, transparent: true, opacity: 1 });
        mats.push(m);
        return m;
    };
    const body = std(ROBOT_COLOR);
    // BoxGeometry face order: +x, -x, +y, -y, +z, -z
    const box = new THREE.Mesh(new THREE.BoxGeometry(L, W, H),
        [std(FRONT_COLOR), body, std(LEFT_COLOR), body, body, body]);
    box.position.set(cx, cy, H / 2);
    group.add(box);
    const edgeMat = new THREE.LineBasicMaterial({ color: 0x111111, transparent: true });
    mats.push(edgeMat);
    const edges = new THREE.LineSegments(new THREE.EdgesGeometry(box.geometry), edgeMat);
    edges.position.copy(box.position);
    group.add(edges);
    // edge strips on the top face so front and left read from overhead too
    const strip = 0.03;
    const front = new THREE.Mesh(new THREE.BoxGeometry(strip, W * 0.9, 0.004), std(FRONT_COLOR));
    front.position.set(cx + L / 2 - strip / 2, cy, H + 0.002);
    group.add(front);
    const left = new THREE.Mesh(new THREE.BoxGeometry(L * 0.8, strip, 0.004), std(LEFT_COLOR));
    left.position.set(cx - 0.05 * L, cy + W / 2 - strip / 2, H + 0.002);
    group.add(left);
    const zTop = H + 0.012;
    const ax = arrow([1, 0, 0], [0, 0, zTop], dims.front_m + 0.16, AXIS_X_COLOR, 0.07, 0.05);
    const ay = arrow([0, 1, 0], [0, 0, zTop], dims.left_m + 0.12, AXIS_Y_COLOR, 0.06, 0.04);
    const az = arrow([0, 0, 1], [0, 0, zTop], 0.18, AXIS_Z_COLOR, 0.05, 0.035);
    for (const a of [ax, ay, az]) {
        a.line.material.transparent = true;
        a.cone.material.transparent = true;
        mats.push(a.line.material, a.cone.material);
        group.add(a);
    }
    const dot = new THREE.Mesh(new THREE.SphereGeometry(0.018, 16, 12),
        new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true }));
    dot.position.set(0, 0, zTop);
    mats.push(dot.material);
    group.add(dot);
    const lf = textSprite('fwd +x', { height_m: 0.055, color: '#ffb0b0', center: [0, 0.5] });
    lf.position.set(dims.front_m + 0.18, 0, zTop);
    const ll = textSprite('left +y', { height_m: 0.055, color: '#a8f0c6', center: [0.5, 0] });
    ll.position.set(0, dims.left_m + 0.14, zTop);
    const lu = textSprite('up +z', { height_m: 0.04, color: '#b8d0ff', center: [0, 0.5] });
    lu.position.set(0.02, 0, zTop + 0.2);
    group.add(lf, ll, lu);
    const note = textSprite('last known pose', { height_m: 0.06, bold: true, color: '#ffd27a', bg: 'rgba(60,20,0,0.8)', center: [0.5, 0] });
    note.position.set(0, 0, H + 0.3);
    note.visible = false;
    group.add(note);
    group.userData.materials = mats;
    group.userData.note = note;
    group.userData.dims = { ...dims };
    group.userData.only3d = [lu];
    return group;
}

// Pose origin on the floor at (x, y); attitude only rotates, never lifts.
export function applyRobotPose(robot, x, y, q) {
    robot.position.set(...fieldToScene(x, y, 0));
    robot.quaternion.set(q[0], q[1], q[2], q[3]);
}

export function setRobotLive(robot, live) {
    for (const m of robot.userData.materials) {
        m.opacity = live ? 1 : DIM_OPACITY;
        m.depthWrite = live;
    }
    robot.userData.note.visible = !live;
}

export function applyCameraPose(camera, controls, pose) {
    camera.position.set(...pose.position);
    controls.target.set(...pose.target);
    camera.up.set(SCENE_UP[0], SCENE_UP[1], SCENE_UP[2]);
    camera.lookAt(controls.target);
    controls.update();
}

export class FieldScene {
    constructor(container) {
        this.container = container;
        this.available = false;
        this.mode = 'overhead';
        this.dims = { inside_x_m: 3.6576, inside_y_m: 3.6576 };
        this.userMoved = false;
        this.robotKey = '';
        this.trailVersion = -1;
        this.needsRender = true;
        this.lastRenderAt = -Infinity;
        this.drawn = { x: NaN, y: NaN, q: [NaN, NaN, NaN, NaN], live: null };
        try {
            this.renderer = new THREE.WebGLRenderer({ antialias: true });
        } catch (e) {
            this.error = 'WebGL unavailable: ' + (e && e.message ? e.message : String(e));
            return;
        }
        this.available = true;
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
        this.renderer.setClearColor(BG_COLOR, 1);
        container.appendChild(this.renderer.domElement);

        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(FOV_OVERHEAD_DEG, 1, 0.02, 200);
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.15;
        this.controls.addEventListener('start', () => {
            this.userMoved = true;
        });
        this.controls.addEventListener('change', () => {
            this.needsRender = true;
        });

        this.scene.add(new THREE.HemisphereLight(0xffffff, 0x3a3f4a, 1.1));
        const sun = new THREE.DirectionalLight(0xffffff, 1.4);
        sun.position.set(2, -3, 5);
        this.scene.add(sun);

        this.fieldGroup = null;
        this.conventions = buildConventions();
        this.scene.add(this.conventions);

        this.robot = null;
        this.trail = null;

        this.resize();
        const later = () => requestAnimationFrame(() => this.resize());
        if (typeof ResizeObserver !== 'undefined') {
            new ResizeObserver(later).observe(container);
        } else {
            window.addEventListener('resize', later);
        }
        this.setView('overhead');
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
        this.needsRender = true;
        if (this.mode === 'overhead' && !this.userMoved) {
            this.resetView();
        }
    }

    setField(field) {
        if (!this.available) {
            return;
        }
        if (this.fieldGroup) {
            this.scene.remove(this.fieldGroup);
            disposeObject(this.fieldGroup);
            this.fieldGroup = null;
        }
        if (field) {
            this.fieldGroup = buildFieldGroup(field);
            this.scene.add(this.fieldGroup);
            this.dims = field.dimensions;
        }
        this.needsRender = true;
        this.resetView();
    }

    // Rebuilds the robot only when its dimensions change.
    setRobotDims(dims) {
        if (!this.available) {
            return;
        }
        const key = JSON.stringify(dims);
        if (key === this.robotKey) {
            return;
        }
        this.robotKey = key;
        let visible = false;
        let pos = null, quat = null, live = true;
        if (this.robot) {
            visible = this.robot.visible;
            pos = this.robot.position.clone();
            quat = this.robot.quaternion.clone();
            live = !this.robot.userData.note.visible;
            this.scene.remove(this.robot);
            disposeObject(this.robot);
        }
        this.robot = buildRobot(dims);
        this.robot.visible = visible;
        if (pos) {
            this.robot.position.copy(pos);
            this.robot.quaternion.copy(quat);
        }
        setRobotLive(this.robot, live);
        this.scene.add(this.robot);
        this.showOnly3d();
        this.drawn.live = live;
        this.needsRender = true;
    }

    showRobot(pose, live) {
        if (!this.available || !this.robot) {
            return;
        }
        if (!pose) {
            if (this.robot.visible) {
                this.robot.visible = false;
                this.needsRender = true;
            }
            return;
        }
        const d = this.drawn;
        const q = pose.q;
        if (!this.robot.visible || d.x !== pose.x || d.y !== pose.y || d.q[0] !== q[0] ||
            d.q[1] !== q[1] || d.q[2] !== q[2] || d.q[3] !== q[3]) {
            this.robot.visible = true;
            applyRobotPose(this.robot, pose.x, pose.y, q);
            d.x = pose.x;
            d.y = pose.y;
            d.q = q;
            this.needsRender = true;
        }
        if (live !== d.live) {
            setRobotLive(this.robot, live);
            d.live = live;
            this.needsRender = true;
        }
    }

    // The trail draws straight from the ring's buffer.
    setTrail(ring) {
        if (!this.available) {
            return;
        }
        if (!this.trail || this.trail.userData.ring !== ring) {
            if (this.trail) {
                this.scene.remove(this.trail);
                this.trail.geometry.dispose();
            }
            const attr = new THREE.BufferAttribute(ring.positions, 3).setUsage(THREE.DynamicDrawUsage);
            const geom = new THREE.BufferGeometry().setAttribute('position', attr);
            this.trail = new THREE.LineSegments(geom, new THREE.LineBasicMaterial({ color: TRAIL_COLOR }));
            this.trail.frustumCulled = false;
            this.trail.userData.ring = ring;
            this.trailVersion = -1;
            this.scene.add(this.trail);
        }
        if (ring.version !== this.trailVersion) {
            this.trailVersion = ring.version;
            this.trail.geometry.attributes.position.needsUpdate = true;
            this.trail.geometry.setDrawRange(0, ring.count * 2);
            this.needsRender = true;
        }
    }

    // 'overhead': top-down, +X right, +Y up, rotation off (left drag pans).
    // 'orbit': free 3D orbit. 'follow': 3D, the target tracks the robot.
    setView(mode) {
        if (!this.available) {
            return;
        }
        this.mode = mode;
        const overhead = mode === 'overhead';
        this.controls.enableRotate = !overhead;
        this.controls.mouseButtons.LEFT = overhead ? THREE.MOUSE.PAN : THREE.MOUSE.ROTATE;
        this.controls.touches.ONE = overhead ? THREE.TOUCH.PAN : THREE.TOUCH.ROTATE;
        this.camera.fov = overhead ? FOV_OVERHEAD_DEG : FOV_3D_DEG;
        this.camera.updateProjectionMatrix();
        this.showOnly3d();
        this.needsRender = true;
        this.resetView();
    }

    // Labels that only make sense from the side (+Z up) are hidden overhead.
    showOnly3d() {
        const on = this.mode !== 'overhead';
        for (const g of [this.conventions, this.robot]) {
            for (const o of (g && g.userData.only3d) || []) {
                o.visible = on;
            }
        }
    }

    resetView() {
        if (!this.available) {
            return;
        }
        this.userMoved = false;
        this.followPending = false;
        let pose;
        if (this.mode === 'overhead') {
            pose = overheadCamera(this.dims, this.camera.fov, this.camera.aspect);
        } else if (this.mode === 'follow' && this.robot && this.robot.visible) {
            const p = this.robot.position;
            pose = {
                position: [p.x + FOLLOW_OFFSET[0], p.y + FOLLOW_OFFSET[1], FOLLOW_OFFSET[2]],
                target: [p.x, p.y, 0],
            };
        } else {
            // follow without a robot yet: wide view until it appears
            this.followPending = this.mode === 'follow';
            pose = orbitCamera(this.dims);
        }
        applyCameraPose(this.camera, this.controls, pose);
        this.needsRender = true;
    }

    // One animation frame: follow, damping, then a render only when
    // something changed (or once a second, so the canvas never sits empty).
    render(now) {
        if (!this.available) {
            return;
        }
        if (this.followPending && this.robot && this.robot.visible) {
            this.resetView();
        }
        if (this.mode === 'follow' && this.robot && this.robot.visible) {
            const p = this.robot.position;
            const dx = p.x - this.controls.target.x;
            const dy = p.y - this.controls.target.y;
            if (dx * dx + dy * dy > 1e-12) {
                this.controls.target.x += dx;
                this.controls.target.y += dy;
                this.camera.position.x += dx;
                this.camera.position.y += dy;
                this.needsRender = true;
            }
        }
        if (this.controls.update()) {
            this.needsRender = true;
        }
        if (!this.needsRender && now - this.lastRenderAt < 1000) {
            return;
        }
        this.needsRender = false;
        this.lastRenderAt = now;
        this.renderer.render(this.scene, this.camera);
    }
}
