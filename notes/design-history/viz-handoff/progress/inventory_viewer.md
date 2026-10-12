# Viewer inventory (explorer report, 2026-09-28, before any change)

Paths under `pi/naviGATR/spectaGATR/` unless noted. Verify before relying on a line number.

## 1. Message handling
- **Socket.** `connect()` (app.js:99-135) opens `ws(s)://host/ws` with `binaryType='arraybuffer'`.
  - Reconnect uses exponential backoff from 500 ms to 5000 ms (21-22, 137-144). Nothing is cleared on reconnect.
  - Text goes through `JSON.parse` on the main thread (116-122), then `onDocument` (165-171). Binary goes to `onBinary`.
- **`onHello`** (173-190): error on a contract change. It rebuilds the scene when the session or configuration digest changed; clears on a session change; calls `setHello` on the camera panel and diagnostics; calls `updateStatus`.
- **`onBinary`** (212-235): u32 header length, JSON header, JPEG. It builds a new `TextDecoder` for every frame, then goes Blob → `createImageBitmap` → redraw.
- **Client → server:** only `{type:'preview',hz,quality,max_width}`, via `sendJson` (app.js:146; diagnostics.js:59-69).
- **`onSnapshot`** (192-210), in order:
  1. `sessionChanged`, then `clearIncompatible` if it changed.
  2. Dataset writes.
  3. `scene.updateSnapshot`.
  4. `cameraPanel.updateSnapshot` (unthrottled, full redraw).
  5. `diagnostics.updateSnapshot` (250 ms throttle).
  6. `linkPanel.update` (250 ms throttle).
  7. `renderLandmarks` (unthrottled).
  8. `setConnection`, then `updateStatus`.

## 2. Rendering
- **Setup.** `FieldScene` (field_scene.js:298-344): WebGLRenderer with antialias and pixel ratio capped at 2; PerspectiveCamera(50,1,0.02,200); OrbitControls with damping 0.12; a dynamic group; `renderer.setAnimationLoop(frame)`, which runs on every vsync.
- **`frame()`** (359-368): the follow target shift allocates `robot.position.clone()` every frame.
- **Trail.** A preallocated `Float32Array` of 4000x3 (657-665).
  - `pushTrailPoint` allocates `[x,y,z]` for each point (678).
  - `copyWithin` is O(N) per point once full (675).
  - `needsUpdate` is set on every snapshot, uploading the whole buffer (761).
- **Robot.** A new `Matrix4` and a new `Vector3` for the label every snapshot (727, 740). `JSON.stringify(body)` builds a key (706).
- **Estimates.** A new `Vector3` for each landmark (924). The label includes the age rounded to 100 ms (918), so `setLabel` (90-104) rebuilds the canvas texture and sprite material about every snapshot.
- **Camera mounts.** A signature from `JSON.stringify(s.K)` for each camera, every snapshot (833).
- **Path.** Rebuilt when the key changes (773), with one SphereGeometry and material per point (791).
- **Static field.** Built once per hello (395-467).

## 3. DOM updates
- `updateStatus` (app.js:264-321) runs every snapshot and every 250 ms: `badgesEl.innerHTML=''` then all badges rebuilt; `readoutEl.innerHTML` set twice.
- `renderLandmarks` (323-373) runs every snapshot: the table is rebuilt, with 2 listeners per row.
- `diagnostics.js fill()`: six tables rebuilt with `innerHTML=''`. It is throttled, but has no trailing flush.
- link_panel.js: every section is rebuilt with `innerHTML` (155, 169, 196, 213, 241, 274).
- camera_panel.js: `renderBadges`, `renderTagList` and the select; `redraw` resets the canvas size when there is no image (290-291).
- **Hidden panels are still updated:** diag collapsed, `#link[hidden]`, and panels scrolled out of view.
- **Timers:**
  - `setInterval` 250 ms: `updateStatus`, `updateAge`, stale check at 1000 ms (376-385).
  - Reconnect `setTimeout`.
  - A 100 ms load gate through top-level await (395-407), used by `tools/viewer_smoke.sh`.

## 4. Features to preserve
- **View:** reset view and top-down; follow; planning toggle. Planning layer: boundary, fixed obstacles, and landmark boxes at the estimate when observed.
- **Landmarks:**
  - nominal ghosts with labels;
  - estimates with a displacement line and opacity fade;
  - tag plates;
  - highlight from table hover.
- **Robot:**
  - the profile footprint when a profile is applied, else RobotBody;
  - placed/unplaced colour;
  - camera mounts with frustums;
  - trail;
  - Brain path (direct or avoiding colour).
- **Scene extras:** hover tooltip with raycast; axes and wall labels; a WebGL-unavailable note.
- **Status bar:** readiness, localization, placement, clock, attitude, field snapshot, rates, warnings, readout.
- **Test hooks:** `#status` data attributes and `window.__navigatr`. The smoke test checks `state`, `live-seen`, `snapshots`, `frames`, `errors`, `attitude` and `webgl`.
- **Camera panel:**
  - camera select;
  - identity-bound overlays (12 kept);
  - tag polygons, corner 0 and pose axes;
  - click and hover selection;
  - tag list with candidates;
  - badges;
  - exposure age.
- **Diagnostics:** sources, workers, inspection row, localization and stillness, diagnostic functions, links, preview budget form, session table, hide toggle.
- **Link panel:** readiness line (link_panel.js:37-122), profile, calibration, wheels, Pico and control operation, events newest first.
- **Landmark table.**

## 5. Attitude
- The model is tilted only when `att.valid` (field_scene.js:719-728): roll and pitch from `robot.attitude`, yaw from the field heading.
- Invalid attitude shows the label "robot (level assumed)" or "(no attitude)", or "(not placed, odometry)".
- Status badge: ok with roll/pitch/age; `assumed_level` as warn; otherwise unavailable. It sets `data-attitude`.
- Trail roll and pitch are unused, and trail z is fixed at 0.015.

## 6. Session and epoch
- **`sessionChanged`:** compares id and reset_count.
- **`clearIncompatible`:** clears the scene session (estimates, trail, path, robot hidden) and the camera panel.
  - It does not reset camera mounts, `robotBodyKey`, the counters, or the diagnostics/link/landmark DOM.
- **Reconnect:** resets nothing.
- **Trail:**
  - The key is `session/reset/odometry_epoch/anchor_revision`; a change clears the trail.
  - Entries from other epochs, or at or before the last `host_ms`, are skipped.
  - Each point is composed through `field_from_odom`, so a placement redraws the trail.

## 7. Structure
- A 3-row grid: status, `#main` (`1fr` plus a 440 px side column, 360 px below 1100 px), and `#diag` (215 px, collapsible).
- There are no tabs.
- Plain CSS; ES modules with an importmap for three; no build step.
- **Embedding:** the CMake glob is `spectaGATR/*.{html,js,css,json,svg}`, top level only, keyed by basename. `vendor/` maps to `third_party/three`.

## 8. Hotspots
1. Full camera panel redraw every snapshot.
2. `renderLandmarks` every snapshot.
3. `updateStatus` every snapshot plus 250 ms.
4. Label sprite re-rasterization.
5. Trail: whole-buffer upload, O(N) shift, and allocation per point.
6. Continuous render and a clone in follow mode.
7. One sphere per path point.
8. Raycast on every mouse move, plus a forced layout.
9. Main-thread `JSON.parse` of the full snapshot; `JSON.stringify` keys; a `TextDecoder` per frame.
10. Throttles without a trailing flush; hidden panels still updated.
11. `axisLength` scans on every camera redraw.
