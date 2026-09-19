// field_assets_gtest.cpp
// The checked-in Override field definition against the game manual: the
// perimeter and display data parse, the nine nominal goal centers sit on
// the manual's A10 grid, the layout has the manual's symmetry, every tag
// mount points outward at a plausible height, and the display-only
// features stay inside the walls. Also the live camera profile: it
// resolves, fails on this host only for the missing libcamera backend,
// and its pipeline runs on the synthetic rig.

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "config/composition.h"
#include "config/config_node.h"
#include "config/field_map.h"
#include "payloads/tag_observations.h"
#include "runtime/register_all.h"
#include "runtime/system.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{

const std::string kConfigDir = NAVIGATR_CONFIG_DIR;

constexpr double kInsideM = 3.5664;   // manual v2.0 sheet A13, portable perimeter
constexpr double kCenterM = kInsideM / 2.0;
constexpr double kInchM   = 0.0254;

bool loadOverrideField(FieldMap& map, std::string& err) {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile((kConfigDir + "/override/field.xml").c_str()) !=
        tinyxml2::XML_SUCCESS) {
        err = "cannot load config/override/field.xml";
        return false;
    }
    return parseFieldMap(ConfigNode{doc.RootElement()}, map, err);
}

bool parseInline(const char* xml, FieldMap& map, std::string& err) {
    tinyxml2::XMLDocument doc;
    if (doc.Parse(xml) != tinyxml2::XML_SUCCESS) {
        err = doc.ErrorStr();
        return false;
    }
    return parseFieldMap(ConfigNode{doc.RootElement()}, map, err);
}

bool startsWith(const std::string& s, const char* prefix) { return s.rfind(prefix, 0) == 0; }

// Goal height class by landmark id: the file's naming convention is part
// of what these tests pin (manual A7 heights).
double expectedHeightFor(const std::string& id) {
    if (startsWith(id, "neutral_goal_0")) {
        return 0.2227;
    }
    if (startsWith(id, "neutral_goal_")) {
        return 0.1465;
    }
    return 0.0825;   // red_goal_* and blue_goal_*
}

const FieldFeatureDecl* featureById(const FieldMap& map, const char* id) {
    for (const FieldFeatureDecl& f : map.features) {
        if (f.id == id) {
            return &f;
        }
    }
    return nullptr;
}

} // namespace

TEST(FieldAssets, DimensionsMatchTheManual) {
    FieldMap    map;
    std::string err;
    ASSERT_TRUE(loadOverrideField(map, err)) << err;
    EXPECT_EQ(map.name, "V5RC Override 2026-2027");
    ASSERT_TRUE(map.dimensions.declared);
    const FieldDimensions& d = map.dimensions;
    // sheet A13: 140.40 in [3566.4 mm] square, 11.54 in [293 mm] wall,
    // 2.00 in [50.8 mm] wall thickness; tile pitch is inside/6
    EXPECT_NEAR(d.inside_x_m, 3.5664, 1e-9);
    EXPECT_NEAR(d.inside_y_m, 3.5664, 1e-9);
    EXPECT_NEAR(d.wall_height_m, 0.293, 1e-9);
    EXPECT_NEAR(d.wall_thickness_m, 0.0508, 1e-9);
    EXPECT_NEAR(d.tile_m, 3.5664 / 6.0, 1e-9);
    EXPECT_NE(d.source.find("A13"), std::string::npos) << d.source;
    EXPECT_FALSE(d.revision.empty());
    EXPECT_FALSE(d.units_note.empty());
}

TEST(FieldAssets, EveryLandmarkHasAVisualOfItsHeightClass) {
    FieldMap    map;
    std::string err;
    ASSERT_TRUE(loadOverrideField(map, err)) << err;
    ASSERT_EQ(map.landmarks.size(), 9u);
    std::map<std::string, std::string> color_by_class;
    for (const LandmarkDecl& lm : map.landmarks) {
        const std::string&        id = lm.id.value;
        const LandmarkVisualDecl& v  = lm.visual;
        ASSERT_TRUE(v.declared) << id;
        EXPECT_EQ(v.shape, "octagonal_prism") << id;
        EXPECT_NEAR(v.height_m, expectedHeightFor(id), 1e-9) << id;
        EXPECT_NEAR(v.base_across_flats_m, 0.1425, 1e-9) << id;
        EXPECT_NEAR(v.top_across_flats_m, 0.0888, 1e-9) << id;
        EXPECT_NEAR(v.tag_plate_width_m, 0.056, 1e-9) << id;
        EXPECT_NEAR(v.tag_plate_height_m, 0.0508, 1e-9) << id;
        EXPECT_NEAR(v.tag_plate_thickness_m, 0.0049, 1e-9) << id;
        EXPECT_EQ(v.color.size(), 7u) << id;
        EXPECT_FALSE(v.note.empty()) << id;
        const std::string cls =
            startsWith(id, "red_") ? "red" : startsWith(id, "blue_") ? "blue" : "neutral";
        const auto seen = color_by_class.find(cls);
        if (seen == color_by_class.end()) {
            color_by_class.emplace(cls, v.color);
        } else {
            EXPECT_EQ(seen->second, v.color) << id;
        }
    }
    ASSERT_EQ(color_by_class.size(), 3u);
    EXPECT_NE(color_by_class["red"], color_by_class["blue"]);
    EXPECT_NE(color_by_class["red"], color_by_class["neutral"]);
    EXPECT_NE(color_by_class["blue"], color_by_class["neutral"]);
}

TEST(FieldAssets, NominalCentersMatchTheManualGrid) {
    FieldMap    map;
    std::string err;
    ASSERT_TRUE(loadOverrideField(map, err)) << err;
    // sheets A10 (positions, audience view, inches) and A16 (tag ids)
    struct Expected {
        const char* id;
        double      x_in;
        double      y_in;
        int         observed_id;
    };
    const Expected expected[] = {
        {"neutral_goal_0_center", 70.20, 70.20, 0},
        {"neutral_goal_1_west", 23.11, 93.75, 1},
        {"neutral_goal_1_east", 117.30, 46.66, 1},
        {"neutral_goal_4_north", 46.66, 117.30, 4},
        {"neutral_goal_4_south", 93.75, 23.11, 4},
        {"red_goal_2_west", 23.11, 46.66, 2},
        {"red_goal_3_south", 46.66, 23.11, 3},
        {"blue_goal_2_east", 117.30, 93.75, 2},
        {"blue_goal_3_north", 93.75, 117.30, 3},
    };
    ASSERT_EQ(map.landmarks.size(), sizeof(expected) / sizeof(expected[0]));
    for (const Expected& e : expected) {
        const LandmarkDecl* lm = map.find(FieldObjectId{e.id});
        ASSERT_NE(lm, nullptr) << e.id;
        EXPECT_NEAR(lm->nominal.x_m, e.x_in * kInchM, 0.001) << e.id;
        EXPECT_NEAR(lm->nominal.y_m, e.y_in * kInchM, 0.001) << e.id;
        EXPECT_NEAR(lm->nominal.heading_rad, 0.0, 1e-12) << e.id;
        ASSERT_EQ(lm->mounts.size(), 4u) << e.id;
        for (const TagMountDecl& m : lm->mounts) {
            EXPECT_EQ(m.observed_id, e.observed_id) << m.instance_id;
        }
    }
}

TEST(FieldAssets, LayoutIsRotationallySymmetricAndAlliancesMirror) {
    FieldMap    map;
    std::string err;
    ASSERT_TRUE(loadOverrideField(map, err)) << err;
    int red = 0, blue = 0, neutral = 0;
    for (const LandmarkDecl& lm : map.landmarks) {
        // 180 degrees about the field center lands on a goal with the same
        // printed id (every id but 0 appears twice, sheet A16)
        const double        tx   = kInsideM - lm.nominal.x_m;
        const double        ty   = kInsideM - lm.nominal.y_m;
        const LandmarkDecl* twin = nullptr;
        for (const LandmarkDecl& other : map.landmarks) {
            if (std::fabs(other.nominal.x_m - tx) < 0.001 &&
                std::fabs(other.nominal.y_m - ty) < 0.001) {
                twin = &other;
                break;
            }
        }
        ASSERT_NE(twin, nullptr) << lm.id.value;
        std::set<int> ids, twin_ids;
        for (const TagMountDecl& m : lm.mounts) {
            ids.insert(m.observed_id);
        }
        for (const TagMountDecl& m : twin->mounts) {
            twin_ids.insert(m.observed_id);
        }
        EXPECT_EQ(ids.size(), 1u) << lm.id.value;
        EXPECT_EQ(ids, twin_ids) << lm.id.value << " vs " << twin->id.value;

        // red is the audience-left (small x) alliance, blue the right; each
        // alliance's goals also sit on its own half in y
        if (startsWith(lm.id.value, "red_")) {
            ++red;
            EXPECT_LT(lm.nominal.x_m, kCenterM) << lm.id.value;
            EXPECT_LT(lm.nominal.y_m, kCenterM) << lm.id.value;
            EXPECT_TRUE(startsWith(twin->id.value, "blue_")) << twin->id.value;
        } else if (startsWith(lm.id.value, "blue_")) {
            ++blue;
            EXPECT_GT(lm.nominal.x_m, kCenterM) << lm.id.value;
            EXPECT_GT(lm.nominal.y_m, kCenterM) << lm.id.value;
            EXPECT_TRUE(startsWith(twin->id.value, "red_")) << twin->id.value;
        } else {
            ++neutral;
        }
    }
    EXPECT_EQ(red, 2);
    EXPECT_EQ(blue, 2);
    EXPECT_EQ(neutral, 5);
    const LandmarkDecl* center = map.find(FieldObjectId{"neutral_goal_0_center"});
    ASSERT_NE(center, nullptr);
    EXPECT_NEAR(center->nominal.x_m, kCenterM, 1e-6);
    EXPECT_NEAR(center->nominal.y_m, kCenterM, 1e-6);
}

TEST(FieldAssets, MountsPointOutwardBetweenFloorAndGoalTop) {
    FieldMap    map;
    std::string err;
    ASSERT_TRUE(loadOverrideField(map, err)) << err;
    std::set<std::string> instance_ids;
    std::size_t           mounts = 0;
    for (const LandmarkDecl& lm : map.landmarks) {
        std::set<std::string> directions;
        for (const TagMountDecl& m : lm.mounts) {
            ++mounts;
            EXPECT_TRUE(instance_ids.insert(m.instance_id).second) << m.instance_id;
            EXPECT_EQ(m.family, "tagCircle21h7") << m.instance_id;
            EXPECT_NEAR(m.detection_size_m, 0.01761272, 1e-12) << m.instance_id;

            // the surface origin sits on exactly one axis of the goal frame,
            // between the top and the base across-flats radii (scale check)
            const Transform3& T       = m.T_landmark_tag_surface;
            const bool        along_x = std::fabs(T.x_m) > 0.01;
            const bool        along_y = std::fabs(T.y_m) > 0.01;
            EXPECT_NE(along_x, along_y) << m.instance_id;
            EXPECT_NEAR(along_x ? T.y_m : T.x_m, 0.0, 1e-9) << m.instance_id;
            const double offset = std::hypot(T.x_m, T.y_m);
            EXPECT_GT(offset, lm.visual.top_across_flats_m / 2.0) << m.instance_id;
            EXPECT_LT(offset, lm.visual.base_across_flats_m / 2.0) << m.instance_id;
            directions.insert(along_x ? (T.x_m > 0 ? "+x" : "-x") : (T.y_m > 0 ? "+y" : "-y"));

            // the surface normal (+x of the tag frame) points the way the
            // offset does (sign check); the 5 degree pitch leaves cos(5) of
            // it in the plane
            const double nx  = T.R.m[0][0];
            const double ny  = T.R.m[1][0];
            const double dot = (nx * T.x_m + ny * T.y_m) / offset;
            EXPECT_GT(dot, 0.99) << m.instance_id;

            // above the tiles, below the goal top
            EXPECT_GT(T.z_m, 0.0) << m.instance_id;
            EXPECT_LT(T.z_m, lm.visual.height_m) << m.instance_id;
        }
        EXPECT_EQ(directions.size(), 4u) << lm.id.value;
    }
    EXPECT_EQ(mounts, 36u);
    EXPECT_EQ(instance_ids.size(), 36u);
}

TEST(FieldAssets, FeaturesParseAndStayInsideThePerimeter) {
    FieldMap    map;
    std::string err;
    ASSERT_TRUE(loadOverrideField(map, err)) << err;
    ASSERT_EQ(map.features.size(), 12u);   // 4 loaders, 4 toggles, 4 tape strips

    std::set<std::string> ids;
    for (const FieldFeatureDecl& f : map.features) {
        EXPECT_TRUE(ids.insert(f.id).second) << f.id;
        EXPECT_TRUE(f.kind == "box" || f.kind == "tape") << f.id;
        EXPECT_EQ(f.color.size(), 7u) << f.id;
        EXPECT_FALSE(f.note.empty()) << f.id;
        // the rotated footprint stays inside the walls, nothing under the tiles
        const double c = std::cos(f.yaw_rad), s = std::sin(f.yaw_rad);
        for (const int sx : {-1, 1}) {
            for (const int sy : {-1, 1}) {
                const double dx = sx * f.size_x_m / 2.0, dy = sy * f.size_y_m / 2.0;
                const double x = f.x_m + c * dx - s * dy;
                const double y = f.y_m + s * dx + c * dy;
                EXPECT_GE(x, -1e-9) << f.id;
                EXPECT_LE(x, kInsideM + 1e-9) << f.id;
                EXPECT_GE(y, -1e-9) << f.id;
                EXPECT_LE(y, kInsideM + 1e-9) << f.id;
            }
        }
        EXPECT_GE(f.z_m - f.size_z_m / 2.0, -1e-9) << f.id;
    }

    // loaders: flush against their alliance wall at the A10 positions,
    // 95 mm into the field, A9 body size
    struct Loader {
        const char* id;
        bool        red;
        double      y_m;
    };
    for (const Loader& l : {Loader{"loader_red_south", true, 0.2906},
                            Loader{"loader_red_north", true, 3.2758},
                            Loader{"loader_blue_south", false, 0.2906},
                            Loader{"loader_blue_north", false, 3.2758}}) {
        const FieldFeatureDecl* f = featureById(map, l.id);
        ASSERT_NE(f, nullptr) << l.id;
        EXPECT_EQ(f->kind, "box") << l.id;
        EXPECT_NEAR(l.red ? f->x_m - f->size_x_m / 2.0 : f->x_m + f->size_x_m / 2.0,
                    l.red ? 0.0 : kInsideM, 1e-9)
            << l.id;
        EXPECT_NEAR(f->y_m, l.y_m, 1e-9) << l.id;
        EXPECT_NEAR(f->size_x_m, 0.095, 1e-9) << l.id;
        EXPECT_NEAR(f->size_y_m, 0.1021, 1e-9) << l.id;
        EXPECT_NEAR(f->size_z_m, 0.5835, 1e-9) << l.id;
        EXPECT_NEAR(f->z_m - f->size_z_m / 2.0, 0.0, 1e-9) << l.id;
    }

    // toggles: centered on each wall, 660.2 mm along it, top at A8's 335.3 mm
    struct Toggle {
        const char* id;
        char        wall;   // n s w e
    };
    for (const Toggle& t : {Toggle{"toggle_north", 'n'}, Toggle{"toggle_south", 's'},
                            Toggle{"toggle_west", 'w'}, Toggle{"toggle_east", 'e'}}) {
        const FieldFeatureDecl* f = featureById(map, t.id);
        ASSERT_NE(f, nullptr) << t.id;
        EXPECT_EQ(f->kind, "box") << t.id;
        const bool along_x = t.wall == 'n' || t.wall == 's';
        EXPECT_NEAR(along_x ? f->x_m : f->y_m, kCenterM, 1e-9) << t.id;
        EXPECT_NEAR(along_x ? f->size_x_m : f->size_y_m, 0.6602, 1e-9) << t.id;
        const double across = along_x ? f->y_m : f->x_m;
        const double half   = (along_x ? f->size_y_m : f->size_x_m) / 2.0;
        const bool   far    = t.wall == 'n' || t.wall == 'e';
        EXPECT_NEAR(far ? across + half : across - half, far ? kInsideM : 0.0, 1e-9) << t.id;
        EXPECT_NEAR(f->z_m + f->size_z_m / 2.0, 0.3353, 1e-9) << t.id;
    }

    // midfield diamond: four 864.8 x 63.5 mm strips tangent to a circle of
    // radius 432.4 mm about the center, at 45 degrees (A11)
    for (const char* id :
         {"midfield_tape_ne", "midfield_tape_nw", "midfield_tape_sw", "midfield_tape_se"}) {
        const FieldFeatureDecl* f = featureById(map, id);
        ASSERT_NE(f, nullptr) << id;
        EXPECT_EQ(f->kind, "tape") << id;
        EXPECT_NEAR(f->size_x_m, 0.8648, 1e-9) << id;
        EXPECT_NEAR(f->size_y_m, 0.0635, 1e-9) << id;
        EXPECT_NEAR(f->z_m, 0.0, 1e-9) << id;
        const double rx = f->x_m - kCenterM, ry = f->y_m - kCenterM;
        EXPECT_NEAR(std::hypot(rx, ry), 0.8648 / 2.0, 1e-4) << id;
        EXPECT_NEAR(std::fabs(f->yaw_rad), std::atan(1.0), 1e-9) << id;
        // the strip runs perpendicular to its radial direction
        EXPECT_NEAR(std::cos(f->yaw_rad) * rx + std::sin(f->yaw_rad) * ry, 0.0, 1e-4) << id;
    }
}

TEST(FieldAssets, UnknownVisualShapeAndBadFeatureColorFailNamingTheAttribute) {
    const char* bad_shape = R"(
<Resource id="f" type="field_map">
  <Landmark id="g">
    <NominalPose calibration_status="verified" x_m="1" y_m="1" heading_deg="0"/>
    <Visual shape="cylinder" height_m="0.1" base_across_flats_m="0.1" top_across_flats_m="0.1"/>
  </Landmark>
</Resource>)";
    FieldMap    map;
    std::string err;
    EXPECT_FALSE(parseInline(bad_shape, map, err));
    EXPECT_NE(err.find("shape"), std::string::npos) << err;
    EXPECT_NE(err.find("Visual"), std::string::npos) << err;

    const char* bad_color = R"(
<Resource id="f" type="field_map">
  <Feature id="tape" kind="tape" x_m="1" y_m="1" z_m="0"
           size_x_m="1" size_y_m="0.06" size_z_m="0" color="red"/>
</Resource>)";
    FieldMap map2;
    err.clear();
    EXPECT_FALSE(parseInline(bad_color, map2, err));
    EXPECT_NE(err.find("color"), std::string::npos) << err;
    EXPECT_NE(err.find("Feature"), std::string::npos) << err;
}

TEST(FieldAssets, LiveCameraProfileResolvesAndNeedsTheLibcameraBackend) {
    ResolvedConfiguration resolved;
    std::string           err;
    ASSERT_TRUE(resolveConfiguration(
        kConfigDir + "/override/diagnostics/live_camera_inspection.xml", resolved, err))
        << err;
    EXPECT_EQ(resolved.id, "override_live_camera_inspection");
    EXPECT_EQ(resolved.files.size(), 4u);   // profile, robot, field, pipeline
    EXPECT_NE(resolved.xml.find("<Inspection"), std::string::npos);
    EXPECT_NE(resolved.xml.find("port=\"8765\""), std::string::npos);
    EXPECT_NE(resolved.xml.find("libcamera_camera"), std::string::npos);
    EXPECT_EQ(resolved.xml.find("<Calibration"), std::string::npos);   // uncalibrated on purpose

    FunctionRegistry functions;
    registerAll(functions);
#if NAVIGATR_HAVE_LIBCAMERA
    // The camera may be unavailable on this machine, but a compiled
    // backend accepts the valid profile and reports device health.
    EXPECT_NE(System::buildFromString(resolved.xml.c_str(), functions, err), nullptr) << err;
#else
    // Missing backend support is a configuration error. Human calibration
    // annotations do not affect profile construction.
    EXPECT_EQ(System::buildFromString(resolved.xml.c_str(), functions, err), nullptr);
    EXPECT_NE(err.find("NAVIGATR_WITH_LIBCAMERA"), std::string::npos) << err;
    EXPECT_NE(err.find("front_camera_device"), std::string::npos) << err;
#endif

    // the diagnostic templates carry the same inspection settings
    for (const char* name :
         {"/override/diagnostics/two_wheel_imu.xml.in",
          "/override/diagnostics/three_wheel_imu.xml.in",
          "/override/diagnostics/two_wheel_imu_camera.xml.in",
          "/override/diagnostics/three_wheel_imu_camera.xml.in"}) {
        tinyxml2::XMLDocument doc;
        ASSERT_EQ(doc.LoadFile((kConfigDir + name).c_str()), tinyxml2::XML_SUCCESS) << name;
        ASSERT_NE(doc.RootElement(), nullptr) << name;
        const auto* inspection = doc.RootElement()->FirstChildElement("Inspection");
        ASSERT_NE(inspection, nullptr) << name;
        EXPECT_STREQ(inspection->Attribute("enabled"), "true") << name;
        EXPECT_STREQ(inspection->Attribute("bind"), "127.0.0.1") << name;
        EXPECT_STREQ(inspection->Attribute("port"), "8765") << name;
    }
}

TEST(FieldAssets, CameraInspectionPipelineRunsOnTheSyntheticRig) {
    // the loader resolves file references relative to the referencing
    // document only, so the checked-in fragments are copied beside a
    // test-local profile
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / "navigatr_field_assets_gtest";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    const auto copy = [&](const char* from, const char* to) {
        std::filesystem::copy_file(kConfigDir + from, dir / to,
                                   std::filesystem::copy_options::overwrite_existing);
    };
    copy("/shared/robots/synthetic_rig.xml", "robot.xml");
    copy("/override/field.xml", "field.xml");
    copy("/shared/pipelines/camera_inspection_only.xml", "pipeline.xml");
    {
        std::ofstream f(dir / "profile.xml", std::ios::binary);
        f << R"(<Configuration id="field_assets_rig_camera_inspection">
  <Loop rate_hz="30"/>
  <Robot file="robot.xml"/>
  <Field file="field.xml"/>
  <Pipeline file="pipeline.xml"/>
</Configuration>)";
    }

    FunctionRegistry functions;
    registerAll(functions);
    std::string err;
    auto system = System::buildFromFile((dir / "profile.xml").string(), functions, err);
    ASSERT_NE(system, nullptr) << err;
    EXPECT_EQ(system->configurationId(), "field_assets_rig_camera_inspection");
    EXPECT_EQ(system->localization().estimatorType(), "noop");

    // step until the rig's camera delivered a frame the detector decoded
    std::shared_ptr<const FieldSnapshot> snapshot;
    const TagObservationSet*             set    = nullptr;
    int64_t                              now_ms = 0;
    for (int i = 0; i < 90 && set == nullptr; ++i) {
        now_ms += 33;
        system->step(hostTime(now_ms));
        snapshot      = system->fieldSnapshot();
        const auto it = snapshot->observations.find(ObservationId{"tag_observations"});
        if (it != snapshot->observations.end()) {
            set = it->second.payload.get<TagObservationSet>();
            if (set != nullptr && set->tags.empty()) {
                set = nullptr;
            }
        }
    }
    ASSERT_NE(set, nullptr) << "no decoded tag within 3 s of rig time";
    EXPECT_EQ(set->camera, SensorId{"front_camera"});

    // the noop association publishes nothing and the estimator never
    // commits: every object stays at its nominal definition
    EXPECT_TRUE(snapshot->associations.empty());
    EXPECT_EQ(system->field().objects.size(), 9u);
    for (const auto& kv : system->field().objects) {
        EXPECT_EQ(kv.second.source, EstimateSource::kFieldMap) << kv.first.value;
        EXPECT_FALSE(kv.second.observed) << kv.first.value;
    }
    EXPECT_FALSE(system->robot().valid);   // the noop estimator places nothing

    // the viewer's detection frame carries the image and the decodes, no trace
    const auto frames = system->detectionFrames();
    const auto frame  = frames.find(SensorId{"front_camera"});
    ASSERT_NE(frame, frames.end());
    EXPECT_TRUE(frame->second->has_observations);
    EXPECT_FALSE(frame->second->has_trace);
    EXPECT_NE(frame->second->y8, nullptr);
    EXPECT_EQ(frame->second->frame_sequence, set->frame_sequence);

    system.reset();
    std::filesystem::remove_all(dir);
}
