# Inventory: field definition, landmarks, world estimation, inspection and viewer (GATR2-Electronics `pi/navigatr`)

Scope: read-only. Paths are relative to `C:/GATR2-Electronics/pi/navigatr/` unless they are shown in full. Anything marked **[speculative]** is a proposal or an inference, not a fact about the code.

---

## 1. `config/override/field.xml`: schema and contents

### 1.1 Root and structure
- Root is `<Resource id="override_field" type="field_map" name="V5RC Override 2026-2027">` (field.xml:2). `id` and `type` are read by the resource store (`src/resources/resource_store.cpp:29-30`). `name` is read by `parseFieldMap` into `FieldMap::name` (field_map.cpp:132-134). The resource store checks no other root attributes, so the implementation is free to add its own.
- Profiles include it with `<Field file="../field.xml"/>`. The resolver wraps a `Resource` root in `<Field>` (config/composition.cpp:249-270) and merges its `Resource` into System Resources (composition.cpp:377-381). Every pipeline refers to it as `resource_id="override_field"`.
- Element types used: `Dimensions` (1), `Feature` (12), `Landmark` (9), and inside each Landmark `NominalPose` (1), `Visual` (1), `TagMount` (4, each with a `PoseOfTagSurfaceInLandmark`). **No `ApproachFrame` exists anywhere in field.xml**, although the parser supports it (field_map.cpp:187-221).
- `calibration_status="provisional"` appears on every NominalPose and TagMount. **The code ignores it.** composition.h:29 says "calibration_status text is ignored", composition.cpp:292 skips it, and no parser reads it (grep finds hits only in composition.cpp and in header comments). The field_map.h:147-149 claim that attributes are "calibration gated" means only that the attributes are required.
- `parseFieldMap` reads children by name (`child("Dimensions")`, `forEach("Feature")`, `forEach("Landmark")`, field_map.cpp:135-159). It does **not** reject unknown children or attributes. Today a new element such as `<CollisionBox>` would be silently ignored by existing binaries, and so would a misspelling. Contrast the strict `onlyChildren` check in apriltag_world_estimator.cpp:19-35.

### 1.2 Coordinate frame (field.xml:3-16; docs/coordinates.md:9-20)
- The origin is the **inside bottom-left corner** of the 3566.4 mm nominal field in the Audience View, on the tile surface.
- +x is audience right, +y points toward the top / 0-degree wall, +z is up. Planar angle is counterclockwise from +x.
- Landmark origins are the plan-view goal centers projected to the floor. Every landmark frame is aligned with the field frame, so every `heading_deg="0"`.
- Tag surface frame: +x is the outward normal, +z is the decoded printed top.
- Estimates are unbounded: a pose outside the field is never clamped (field_state.h:41-44; world_estimation_gtest `EstimatesAreUnboundedOutsideTheNominalField`).

### 1.3 Field bounds: `Dimensions` (field.xml:37-45)
`inside_x_m="3.5664" inside_y_m="3.5664" wall_height_m="0.293" wall_thickness_m="0.0508" tile_m="0.5944"`, plus `source`, `revision="manual v2.0 (2026-09-03); CAD zip Last-Modified 2026-04-27"` and `units_note`.
- The drivable interior is x, y in [0, 3.5664]. The viewer draws the walls outside that square (field_scene.js:436-449).
- The alternate metal perimeter is 3568.7 mm. Which perimeter a real field uses is still open (docs/field_assets.md:218-219).

### 1.4 Features (static, unobserved, display only)
`kind` is `box` or `tape` (field_map.cpp:70-73). `x_m/y_m/z_m` is the center in the field frame (field_map.h:114). `yaw_deg` is optional and defaults to 0.

| id | kind | center x,y,z (m) | size x,y,z (m) | yaw | horizontal extent (AABB, m) | lines |
|---|---|---|---|---|---|---|
| loader_red_south | box | 0.0475, 0.2906, 0.29175 | 0.095, 0.1021, 0.5835 | 0 | x[0, 0.095] y[0.23955, 0.34165] | 52-55 |
| loader_red_north | box | 0.0475, 3.2758, 0.29175 | same | 0 | x[0, 0.095] y[3.22475, 3.32685] | 56-59 |
| loader_blue_south | box | 3.5189, 0.2906, 0.29175 | same | 0 | x[3.4714, 3.5664] y[0.23955, 0.34165] | 60-63 |
| loader_blue_north | box | 3.5189, 3.2758, 0.29175 | same | 0 | x[3.4714, 3.5664] y[3.22475, 3.32685] | 64-67 |
| toggle_north | box | 1.7832, 3.5404, 0.3093 | 0.6602, 0.052, 0.052 | 0 | x[1.4531, 2.1133] y[3.5144, 3.5664], z[0.2833, 0.3353] | 75-78 |
| toggle_south | box | 1.7832, 0.026, 0.3093 | same | 0 | x[1.4531, 2.1133] y[0, 0.052] | 79-82 |
| toggle_west | box | 0.026, 1.7832, 0.3093 | 0.052, 0.6602, 0.052 | 0 | x[0, 0.052] y[1.4531, 2.1133] | 83-86 |
| toggle_east | box | 3.5404, 1.7832, 0.3093 | same | 0 | x[3.5144, 3.5664] y[1.4531, 2.1133] | 87-90 |
| midfield_tape_ne | tape | 2.08895, 2.08895, 0 | 0.8648, 0.0635, 0 | -45 | AABB [1.7607, 2.4172] both axes | 102-105 |
| midfield_tape_nw | tape | 1.47745, 2.08895, 0 | same | 45 | x[1.1492, 1.8057] y[1.7607, 2.4172] | 106-109 |
| midfield_tape_sw | tape | 1.47745, 1.47745, 0 | same | -45 | [1.1492, 1.8057] both axes | 110-113 |
| midfield_tape_se | tape | 2.08895, 1.47745, 0 | same | 45 | x[1.7607, 2.4172] y[1.1492, 1.8057] | 114-117 |

- Tape is a floor marking with zero height, and the robot drives over it.
- The toggles sit about 283-335 mm above the floor on the walls and protrude 52 mm into the field. Whether they obstruct a robot depends on its height. Their triangular section is drawn as a box (field.xml:70-73).

### 1.5 Landmarks: semantic ids vs printed AprilTag ids
Every landmark has these properties:
- `Visual shape="octagonal_prism"` with `base_across_flats_m=0.1425`, `top_across_flats_m=0.0888`, and a tag plate of 0.056 x 0.0508 x 0.0049 m.
- Four TagMounts on the axis faces, all `family="tagCircle21h7"` with `detection_size_m="0.01761272"`. `PoseOfTagSurfaceInLandmark` is at ±0.04863581744 m on one axis, `pitch_deg=-5`, and `yaw` of 0/90/180/-90 for the east/north/west/south faces.
- Mount `z_m` depends on the height class: 0.0378522596 for alliance goals, 0.1018522596 for short neutral goals, 0.1780522596 for the tall center goal.
- `instance_id` is `<landmark>_<face>_tag`, for example `neutral_goal_4_north_east_tag`.

| Landmark id (semantic) | nominal x, y (m) | printed `observed_id` | height (m) | color | AABB (flats on axes), m | lines |
|---|---|---|---|---|---|---|
| neutral_goal_4_north | 1.185141466, 2.979317068 | 4 | 0.1465 | #2b2b2b | x[1.1139, 1.2564] y[2.9081, 3.0506] | 120-197 |
| blue_goal_3_north | 2.381258534, 2.979317068 | 3 | 0.0825 | #00a3e0 | x[2.3100, 2.4525] y[2.9081, 3.0506] | 200-277 |
| neutral_goal_1_west | 0.587082932, 2.381258534 | 1 | 0.1465 | #2b2b2b | x[0.5158, 0.6583] y[2.3100, 2.4525] | 280-357 |
| blue_goal_2_east | 2.979317068, 2.381258534 | 2 | 0.0825 | #00a3e0 | x[2.9081, 3.0506] y[2.3100, 2.4525] | 360-437 |
| neutral_goal_0_center | 1.7832, 1.7832 | 0 | 0.2227 | #2b2b2b | [1.7120, 1.8545] both axes | 440-517 |
| red_goal_2_west | 0.587082932, 1.185141466 | 2 | 0.0825 | #d50032 | x[0.5158, 0.6583] y[1.1139, 1.2564] | 520-597 |
| neutral_goal_1_east | 2.979317068, 1.185141466 | 1 | 0.1465 | #2b2b2b | x[2.9081, 3.0506] y[1.1139, 1.2564] | 600-677 |
| red_goal_3_south | 1.185141466, 0.587082932 | 3 | 0.0825 | #d50032 | x[1.1139, 1.2564] y[0.5158, 0.6583] | 680-757 |
| neutral_goal_4_south | 2.381258534, 0.587082932 | 4 | 0.1465 | #2b2b2b | x[2.3100, 2.4525] y[0.5158, 0.6583] | 760-837 |

**Can printed ids repeat? Yes, by design.**
- field_map.h:7-11: "A printed tag id is not a unique identity: several physical mounts may share one observed_id".
- The parser checks only `observed_id >= 0` (field_map.cpp:254-258). Uniqueness is enforced only on `Landmark id` (field_map.cpp:170-174), `TagMount instance_id` (map-wide, 235-246), `ApproachFrame id` (map-wide, 198-209) and `Feature id` (147-153).
- In this file, id 0 appears on 4 mounts of one goal. Ids 1-4 each appear on 8 mounts across two goals related by a 180-degree rotation about the field center (docs/field_assets.md:113-117; tested at field_assets_gtest.cpp:172-224).
- Totals: 9 landmarks, 36 mounts, 36 unique instance ids (field_assets_gtest.cpp:266-267).

**Maximum horizontal extent of the goals.** This is derived from the display `Visual`.
- The base octagon is 0.1425 m across flats, so its circumradius is 0.1425 / (2 cos 22.5°) = **0.07712 m** (bounding circle diameter 0.1542 m).
- With flats on the axes, the AABB is 0.1425 x 0.1425 m (±0.07125). Neither the XML nor the C++ declares the octagon orientation. The flats-on-axes convention exists only in the viewer (`field_scene.js:131-137`: "rotateZ(pi/8) puts flats on the axes") and in the fact that the mounts sit on axis faces (field_assets.md:121-122).
- Tag plates reach at most 51.4 mm from center (field_assets.md:127), inside the base footprint.

### 1.6 Trustworthiness
- **Used by estimation and association (calibration-critical):** `NominalPose` and `TagMount`. They come from the CAD parse and the manual grid, and are within 0.13 mm of the manual A10 grid (field_assets.md:99-111). The manual tolerance is ±1 in. Everything is `provisional`: "remains provisional until checked against physical field elements" (field.xml:18-21). Nothing has been physically measured. The tag family and size are inferred, not stated in the manual (field_assets.md:146-163). The tag surface offset, height and slope come from the CAD parse, and the parse of nested transforms is called "the weakest link" (field_assets.md:141-144).
- **Display only:** `Dimensions`, `Feature` and `Visual` (field.xml:23-26: "Estimation never reads them; a wrong value there moves a picture, not an estimate"; field_map.h:13-17, 52-53, 96-98). The sources are manual Appendix A and the CAD parse. The known-uncertain items (field_assets.md:208-219) are:
  - the toggle height reference edge
  - the tape reference edge (864.8 mm vs 845.8 mm centerline)
  - the perimeter variant
  - the omitted diagonal autonomous lines
- **Consumers of display data:** the viewer (hello), `field_assets_gtest`, and `SyntheticRig`. The rig reads only `landmarks`/`mounts`/`nominal` (synthetic_rig.cpp:39-54, 76-90), not display data.
- **No element has collision semantics today.** The only footprint logic anywhere is a test assertion that feature footprints stay inside the walls (field_assets_gtest.cpp:282-295). `RobotBody` is cosmetic (inspection_config.h:146-152). On the Brain, the only mention is `brain/investiGATR/include/investigatr/navigator.h:4`: "Forward driving only, no obstacle routing."

---

## 2. FieldMap C++ API (`src/config/field_map.h`)

```cpp
struct TagMountDecl {                 // :39-45
    std::string instance_id;          // globally unique physical mount name
    std::string family;               // e.g. tag36h11
    int         observed_id = -1;     // what the detector reports
    double      detection_size_m = 0.0;   // detector corner edge, not the sticker
    Transform3  T_landmark_tag_surface;
};
struct ApproachFrameDecl { FrameId id; Transform3 T_landmark_approach; };   // :47-50
struct LandmarkVisualDecl {           // :54-68, display only
    bool declared = false; std::string shape;   // octagonal_prism | box
    double height_m, base_across_flats_m, top_across_flats_m, size_x_m, size_y_m, size_z_m,
           tag_plate_width_m, tag_plate_height_m, tag_plate_thickness_m;
    std::string color, note;
};
struct LandmarkDecl {                 // :70-94
    FieldObjectId id; Pose2D nominal;   // T_field_landmark
    std::vector<ApproachFrameDecl> approaches; std::vector<TagMountDecl> mounts;
    LandmarkVisualDecl visual;
    const ApproachFrameDecl* findApproach(const FrameId&) const;
    const TagMountDecl*      findMount(const std::string& instance_id) const;
};
struct FieldDimensions {              // :99-109, display only
    bool declared = false; double inside_x_m, inside_y_m, wall_height_m, wall_thickness_m,
    tile_m /*0 = undeclared*/; std::string source, revision, units_note;
};
struct FieldFeatureDecl {             // :111-119
    std::string id, kind /*box|tape*/; double x_m, y_m, z_m, size_x_m, size_y_m, size_z_m,
    yaw_rad; std::string color, note;
};
struct FieldMap {                     // :121-145
    std::string name; FieldDimensions dimensions;
    std::vector<FieldFeatureDecl> features; std::vector<LandmarkDecl> landmarks;
    const LandmarkDecl* find(const FieldObjectId&) const;
    const LandmarkDecl* findMountOwner(const std::string& instance_id) const;
};
bool parseFieldMap(const ConfigNode& node, FieldMap& out, std::string& err);   // :150
```

Supporting types:
- `Pose2D {x_m, y_m, heading_rad}` (math/transforms.h:21-25).
- `FieldObjectId = TypedId<FieldObjectIdTag>` (core/ids.h:44).
- Poses are parsed with `parsePlanarPose` / `parseTransform3` (config/pose3_config.h). Every attribute is required, degrees are converted to radians, and R = Rz·Ry·Rx.
- The resource factory `make_field_map` (impl/resources/field_map_resource.cpp:8-16) publishes `ResourceInstance::asContract<const FieldMap>`. Consumers call `require<const FieldMap>(id)`.

Consumers of `FieldMap`:
- `AprilTagWorldEstimator`
- `LandmarkEstimator`
- `TagMountAssociation`
- `target_set` resource (validates landmark, approach frame and allowed mount)
- `ConfiguredTargetResolution`
- `SyntheticRig`
- `inspection_document.cpp` (hello, plus nominal lookup in snapshot)

`brain_link_publisher.cpp` includes the header but **does not validate `<FieldObject object_id>` against the map** (brain_link_publisher.cpp:111-139). A typo there yields source `none` at runtime.

Target data (`src/resources/target_set.h`, `target_set_resource.cpp`):
- `TargetDecl {id, wire_id, kind, landmark, approach_frame, controlled_frame, desired_controlled_in_approach, T_landmark_approach, T_robot_controlled, vision; delta; resolveFromLandmark()}` (target_set.h:63-88).
- **No checked-in profile configures a `target_set`, `configured_targets`, an `ApproachFrame`, or a Publishing `<FieldObject>` mapping.** Every pipeline uses `<TargetResolution type="noop"/>`; see for example config/shared/pipelines/parallel_wheels_bno08x_camera.xml:9-12, 46. These features are exercised only in tests (vision_targets_gtest, brain_io_gtest, brain_link_e2e_gtest, geometry_resources_gtest).

---

## 3. Landmark estimates: all landmarks vs the selected one; nominal vs observed; age and status

**State type** (`src/state/field_state.h`):
```cpp
enum class EstimateSource : uint8_t { kNone = 0, kFieldMap, kObserved };   // :25-29
struct FieldObjectState {                                                  // :40-60
    FramedPose2D pose;            // frame "field", derived under anchor_revision
    double confidence = 0.0; bool valid = false;
    bool observed = false;        // updated by evidence THIS invocation
    MonotonicTime lastObservedAt; EstimateSource source = kNone;
    Pose2D T_odom_object; uint64_t odometry_epoch = 0; uint64_t anchor_revision = 0;
    std::string last_source, last_feature; uint32_t last_source_sequence = 0;
};
struct FieldState { std::unordered_map<FieldObjectId, FieldObjectState, FieldObjectId::Hash> objects; };
```
The per-invocation publication is `FieldSnapshot {FieldState field; ObservationMap observations; AssociationMap associations; MonotonicTime at; uint64_t invocation; FunctionStatus status; std::string diagnostic;}` (runtime/system.h:88-96).

**All configured landmarks are held, not only the selected one.**
- `LandmarkEstimator::run` (impl/world_estimation/landmark_estimator.cpp:42-166) seeds every `field_->landmarks` entry that is missing with `source=kFieldMap`, `confidence=0.5`, `valid=true`, and pose = nominal (:49-61).
- World estimation is target-blind. It runs every invocation whether or not a target exists (contracts/field_estimation.h:19-22; tag_mount_association.h:8-12).
- Association scores every observation against every mount with the same family and id across the whole map (tag_mount_association.cpp:311-366). The prior is the current estimate if valid, else the nominal (:314-318).
- With the `noop` estimator, `FieldState.objects` stays **empty** (world_estimation_gtest.cpp:519-540).

**Nominal vs observed.**
- The nominal is the immutable `LandmarkDecl::nominal`. An entry reports `source=kFieldMap` until evidence folds in.
- Evidence becomes `kObserved`, stored in the odometry frame as `T_odom_object` and tagged with `odometry_epoch` and `anchor_revision`.
- The field pose is `compose(field_from_odom, T_odom_object)` (:151-152).
- On an anchor change it is re-expressed (:81-85). On an odometry epoch change the entry **reverts to nominal** with confidence 0.5 (:68-80).
- Fusion within one invocation is a confidence-weighted mean with a circular heading mean (:98-132). Across invocations it uses `blend` (0..1, default 1.0, meaning the latest replaces) (:145-149).
- `commit="never"` publishes evidence but never moves the state (:88-90).
- Evidence confidence is `1 - score/(2*max_translation_error_m)` (tag_mount_association.cpp:418). This is an algorithm score, not a covariance (landmarks.md:76).

**Age and status.**
- `observed` is true only in the invocation that folded evidence.
- `lastObservedAt` is the host time of the newest folded evidence.
- There is **no age-based expiry** (landmarks.md:28-31).
- `pose.measuredAt` is set to `now` when an entry is seeded or reset (:56, :78), so it is not an observation age.
- Inspection computes `age_ms` from `lastObservedAt` (null for field_map entries) and `displacement_m`/`heading_error_deg` vs nominal (inspection_document.cpp:707-762).

**The "selected" landmark exists only outside field estimation.**
- `CommandState.object_requested/object_wire_id/object_sequence` holds the Brain selection.
- `TargetState` holds a single active target (state/target_state.h:101-115; configured_targets.cpp).
- The Brain link publishes **exactly one** landmark per GET_STATE, the selected wire id mapped through `<FieldObject object_id wire_id>` (brain_link_publisher.cpp:228-259). The rules are:
  - observed only if `o.odometry_epoch == robot.odometry_epoch`, recomputed under the current anchor, with `landmark_age_ms`
  - otherwise nominal with age 0
  - otherwise source none
- Wire struct `gatr2::BrainState` has one `landmark_id / landmark_source / lm_x_mm / lm_y_mm / lm_heading_cdeg / landmark_age_ms` (common/frames.h:152-167). The max frame is 128 bytes (common/frames.h:19).
- **[speculative arithmetic]** Nine landmarks at about 16 bytes each (id, source, 3x i32, u16 age) come to about 144 bytes, which exceeds one frame. Publishing all landmarks therefore needs a new op or chunking.
- **Nothing today sends all landmark estimates to the Brain. Only the inspection snapshot exposes all of them.**

---

## 4. Inspection document and viewer

**Contract.**
- `kInspectionContract = "navigatr.inspect/1"` (inspection/inspection_document.h:31).
- Functions: `helloDocument(const System&, MonotonicTime)`, `snapshotDocument(const System&, const InspectionServiceStats&, MonotonicTime, std::size_t trail_max_entries = 300)`, `frameHeaderDocument(...)` (:33-40).
- The documents are built only from published immutable snapshots (runtime/inspection_state.h:1-5).
- The service (inspection/inspection_service.{h,cpp}) sends `hello` **once per WebSocket open** (inspection_service.cpp:401), then `snapshot` at `snapshot_hz`. It also serves `/api/hello` and `/api/snapshot`.
- Doc: docs/inspection.md:94-196.

**hello** (inspection_document.cpp:374-593):
- `configuration {id, name, digest (hex64), loop_rate_hz}` (:385-391)
- `robot_body` (display, from `<Inspection><RobotBody>`, inspection_config.h:165-171)
- `fields[]`, one per FieldMap resource (:409-509): `resource_id, name, dimensions|null, features[], landmarks[] {id, nominal, visual|null, mounts[] {instance_id, family, observed_id, detection_size_m, T_landmark_tag}, approaches[]}`
- `tag_families`, `cameras[]`, `camera_sensors[]`, `localization`, `warnings[]`

**snapshot** (:595-868):
- `robot` (odom, field, field_from_odom, epoch/anchor, attitude, covariance)
- `localization`, `trail[]` (odometry frame)
- `field_snapshot {invocation, at_host_ms, age_ms, status, diagnostic}`
- `field_objects[]`: every FieldState entry sorted by id, with `valid, observed, source, confidence, frame, pose, nominal|null, heading_error_deg, displacement_m, anchor_revision, odometry_epoch, last_observed_host_ms, age_ms, last_source, last_feature, last_source_sequence`
- `detection_frames[]` (with association trace and candidates)
- `target` (`T_odom_robot_target` only, **odometry frame**, :771-788), `command`, `sources[]`, `workers`, `diagnostics`

**How the viewer draws them** (viewer/field_scene.js):
- `buildField(hello)` (:354-414) builds:
  - the floor, tile grid and walls from `dimensions` (:416-450)
  - features as boxes, tape at least 3 mm thick (:467-479)
  - landmark "ghost" bodies at nominal with tag plates (:392-407; geometry :130-151)
  - wall labels inferred from the `red_`/`blue_` id prefixes (:481-504)
- `updateSnapshot` (:585-648) draws:
  - the robot box from `hello.robot_body` at `robot.field` (tilt only if attitude is valid)
  - the trail re-expressed via the current `field_from_odom`
  - the camera mounts and frustum
- `updateEstimates` (:706-768) draws solid bodies **only for `source === 'observed'`**, a nominal-to-estimate line, and an age fade.
- app.js `renderLandmarks` (:309-359) tables every field object, plus configured landmarks with no estimate.
- A field rebuild happens when the hello session or `configuration.digest` changes (app.js:170).
- The contract string is compared and a change is recorded as an error, not rejected (app.js:167-169).

**Not present:**
- No planned path, obstacle or collision footprint, or keep-out area.
- No robot footprint other than the cosmetic `RobotBody`.
- No drawing of the target pose. The target appears only as text `target_id status` in the diagnostics table (diagnostics.js:153-160).
- landmarks.md:102-104 says the latched target is "re-expressed in the field for inspection". The code emits only the odometry-frame pose (inspection_document.cpp:784) and the viewer never uses it, so the doc and the code disagree.

**How a new section is added** (pattern observed in the code):
1. A worker publishes an immutable snapshot on `System` (like `FieldSnapshot`/`ReportingSnapshot`, runtime/system.h:88-104).
2. `helloDocument`/`snapshotDocument` emit it with `JsonWriter` (`key/field/beginObject/beginArray/value/null/fieldNull`, inspection/json_writer.h; non-finite values become null).
3. Document the fields in docs/inspection.md.
4. Consume them in app.js `onHello`/`onSnapshot`, then in `FieldScene`. JS ignores unknown keys, so additions do not break older viewers.
5. Test them in tests/inspection_document_gtest.cpp (`validJson` plus `find` substring checks, :360-443).

The viewer is compiled into the binary via cmake/embed_assets.cmake (CMakeLists.txt:129-160). No rule exists for when to bump `/1` on additive changes.

**Could it expose a planned path and obstacle footprints?**
- Static field collision boxes fit naturally in `hello.fields[]`.
- Landmark-attached boxes can be drawn at `field_objects[].pose`, the same way estimate bodies reuse `landmarkDecls` (field_scene.js:715-727).
- A planned path, or a Brain-sent RobotProfile footprint, originates at runtime on the Brain, so it would need Brain-to-Pi transfer and a snapshot section, or a re-sent hello. Hello is static per connection today. **[speculative]**

---

## 5. Existing map identity or version notions

| Notion | Where | Nature |
|---|---|---|
| Resource `id="override_field"` | field.xml:2; pipeline refs | config-local handle, not a version |
| `FieldMap::name` "V5RC Override 2026-2027" | field_map.h:122; hello `fields[].name` | display name, may be empty |
| `Dimensions source`/`revision` | field.xml:43-44; field_map.cpp:50-53 | provenance text for display. Required only if `Dimensions` is present, and `Dimensions` is optional |
| Configuration digest | composition.h:30, 45, 60-61; composition.cpp:204-205, 411-; system.cpp:71-75; hello `configuration.digest`; app/main.cpp:102 | FNV-1a 64 over **every** profile file's canonical path and raw bytes |
| `anchor_revision`, `odometry_epoch`, `pi_instance`, session | frames.h; landmarks.md | localization and session identity, **not** map identity |

Properties of the configuration digest:
- It covers the whole profile (robot, pipeline and field), so it is not a field identity.
- It changes when comments, whitespace or the file path change.
- It is 0 for `buildFromString`.
- It is not on the Brain wire.

**There is no field-specific hash, no revision counter, and no map id on the Brain wire.** The Brain has no field map at all (brain/investiGATR and communiGATR contain no field or obstacle data).

---

## 6. Proposal: collision boxes and a stable map identity **[speculative]**

The proposal follows the existing style: `_m` and `_deg` suffixes, `id` plus `note`, required numeric attributes, an ignored `calibration_status` annotation, and strict parsing in `parseFieldMap`.

**Design rule taken from the codebase:** display data must never silently become behavior (field.xml:23-26). Collision geometry should therefore be a **separate, explicitly planning-owned element**. It should not be a flag on `Feature`/`Visual`, and the planner should not reuse `Dimensions` implicitly.

```xml
<Resource id="override_field" type="field_map" name="V5RC Override 2026-2027"
          map_id="v5rc_override_2026_27" revision="1">
    <!-- drivable interior for planning; test pins it equal to Dimensions inside_* -->
    <Boundary min_x_m="0" min_y_m="0" max_x_m="3.5664" max_y_m="3.5664"/>

    <!-- static, field frame: planar rectangle, center + size + yaw -->
    <CollisionBox id="loader_red_south_body" calibration_status="provisional"
        x_m="0.0475" y_m="0.2906" size_x_m="0.095" size_y_m="0.1021" yaw_deg="0"
        note="manual A9, A10"/>

    <Landmark id="neutral_goal_0_center">
        <NominalPose .../>
        <!-- landmark frame: follows the landmark estimate or nominal -->
        <CollisionBox id="neutral_goal_0_center_body"
            x_m="0" y_m="0" size_x_m="0.1542" size_y_m="0.1542" yaw_deg="0"
            note="base octagon circumscribed square"/>
        ...
    </Landmark>
</Resource>
```

**C++ additions:**
- Add `struct CollisionBoxDecl { std::string id; Pose2D center; double size_x_m, size_y_m; std::string note; };`.
- Add `std::vector<CollisionBoxDecl> collision_boxes;` in both `FieldMap` (field frame) and `LandmarkDecl` (landmark frame).
- Add an optional `struct FieldBoundary`.
- Parse rules: require `id/x_m/y_m/size_x_m/size_y_m`, make `yaw_deg` optional (as for `Feature`), require positive sizes, and enforce map-wide id uniqueness.
- When introducing these, add an unknown-child check to `parseFieldMap` so typos fail loudly (today they are silently ignored).

**Other decisions:**
- Keep z bands (for example the toggles at 283-335 mm) out until needed. Inflation for robot footprint and margin belongs to the planner and RobotProfile, not the field.
- Expose the data in hello as `fields[].boundary`, `fields[].collision_boxes[]` and `landmarks[].collision_boxes[]`.
- In the viewer, draw them as wireframe rectangles. Landmark boxes are drawn at the estimate when `observed`, else at nominal.
- Add a field_assets test that every box stays inside `Boundary` and that goal boxes contain the `Visual` base octagon.

**Map identity:**
- `map_id` (string) and `revision` (integer, human-bumped) go on the Resource root, which already carries implementation-owned attributes (`name`).
- Add a computed `geometry_digest` (uint64) in `FieldMap`, produced by parse from a **canonical serialization of the parsed, quantized semantic content**. The content is the landmark ids, nominal poses, mounts (if the Brain needs them), boundary and collision boxes, in declaration order. Quantize to wire units such as mm and centidegrees, so the Brain can recompute the digest over a received chunked snapshot and verify reassembly.
- Exclude display-only data (colors, notes, Visual, Feature) from this digest, so a cosmetic edit does not invalidate a Brain-side map.
- The FNV-1a 64 helper `contentDigest` already exists (composition.h:61). Putting the canonical encoder and hash in `common/` would let the Pi and the Brain share them.
- Report `map_id`, `revision` and `geometry_digest` in `hello.fields[]` and in the map-transfer header.

---

## Key files (absolute)
- C:/GATR2-Electronics/pi/navigatr/config/override/field.xml
- C:/GATR2-Electronics/pi/navigatr/src/config/field_map.h, field_map.cpp, pose3_config.h, composition.h, composition.cpp
- C:/GATR2-Electronics/pi/navigatr/src/impl/resources/field_map_resource.{h,cpp}, target_set_resource.{h,cpp}
- C:/GATR2-Electronics/pi/navigatr/src/resources/target_set.h
- C:/GATR2-Electronics/pi/navigatr/src/impl/world_estimation/landmark_estimator.{h,cpp}, apriltag_world_estimator.{h,cpp}, tag_mount_association.{h,cpp}
- C:/GATR2-Electronics/pi/navigatr/src/contracts/field_estimation.h, target_resolution.h
- C:/GATR2-Electronics/pi/navigatr/src/state/field_state.h, target_state.h
- C:/GATR2-Electronics/pi/navigatr/src/payloads/field_object_evidence.h
- C:/GATR2-Electronics/pi/navigatr/src/impl/target_resolution/configured_targets.{h,cpp}
- C:/GATR2-Electronics/pi/navigatr/src/impl/publishing/brain_link_publisher.{h,cpp}
- C:/GATR2-Electronics/pi/navigatr/src/inspection/inspection_document.{h,cpp}, inspection_service.{h,cpp}, json_writer.h
- C:/GATR2-Electronics/pi/navigatr/src/runtime/inspection_state.h, inspection_config.h, system.h, system.cpp
- C:/GATR2-Electronics/pi/navigatr/viewer/field_scene.js, app.js, diagnostics.js, camera_panel.js, transforms.js
- C:/GATR2-Electronics/pi/navigatr/docs/field_assets.md, landmarks.md, inspection.md, coordinates.md
- C:/GATR2-Electronics/pi/navigatr/tests/field_assets_gtest.cpp, world_estimation_gtest.cpp, inspection_document_gtest.cpp
- C:/GATR2-Electronics/common/frames.h (BrainState, kMaxFrameLen)