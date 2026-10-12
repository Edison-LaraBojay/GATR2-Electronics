#!/usr/bin/env python3
"""Apply the world estimation change (plan/world_estimation_plan.md) to an
ae457ee-layout naviGATR copy. Prototype only: every replacement must match
exactly once. New files come from scripts/we_proto/.

Usage: py -3 we_apply_proto.py <path to pi/naviGATR>
"""

import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROTO = os.path.join(HERE, "we_proto")


def edit(root, rel, pairs):
    path = os.path.join(root, rel)
    with open(path, encoding="utf-8", newline="") as f:
        text = f.read()
    for old, new in pairs:
        n = text.count(old)
        if n != 1:
            sys.exit(f"{rel}: expected 1 match, found {n}:\n{old}")
        text = text.replace(old, new)
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(text)
    print("edited", rel)


def copy(root, rel):
    dst = os.path.join(root, rel)
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copyfile(os.path.join(PROTO, rel), dst)
    print("added ", rel)


def main():
    root = sys.argv[1]

    # ---- new files
    for rel in ["src/math/units.h", "src/state/landmark_error.h",
                "src/impl/world_estimation/tag_association_trace.h",
                "tests/landmark_error_gtest.cpp"]:
        copy(root, rel)

    # ---- field state
    edit(root, "src/state/field_state.h", [
        ("// invalidates it back to the map nominal. A map-seeded entry is field\n"
         "// geometry and has no odometry-frame value.\n",
         "// invalidates it back to the map nominal. A map-seeded entry is field\n"
         "// geometry and has no odometry-frame value.\n"
         "//\n"
         "// A mapped entry also carries its map nominal and the LandmarkError of\n"
         "// its field pose from it, written together with the pose.\n"),
        ("#include <cstdint>\n#include <string>\n#include <unordered_map>\n\n"
         "#include \"core/ids.h\"\n#include \"math/transforms.h\"\n",
         "#include <cstdint>\n#include <optional>\n#include <string>\n#include <unordered_map>\n\n"
         "#include \"core/ids.h\"\n#include \"math/transforms.h\"\n"
         "#include \"state/landmark_error.h\"\n"),
        ("    FramedPose2D pose;   // frame is field, derived under anchor_revision\n",
         "    FramedPose2D pose;   // frame is field, derived under anchor_revision\n"
         "    std::optional<Pose2D> nominal;   // estimator's map pose, empty when not in its map\n"
         "    LandmarkError         error;     // pose minus nominal, field axes; zero without one\n"),
    ])

    # ---- landmark estimator: error derived at every pose site
    edit(root, "src/impl/world_estimation/landmark_estimator.cpp", [
        ("namespace navigatr\n{\n\n",
         "namespace navigatr\n{\n\nnamespace\n{\n\n"
         "// field pose and its error from nominal, always written together\n"
         "void setFieldPose(FieldObjectState& obj, const Pose2D& pose) {\n"
         "    obj.pose.pose = pose;\n"
         "    obj.error     = obj.nominal ? landmarkError(*obj.nominal, pose) : LandmarkError{};\n"
         "}\n\n} // namespace\n\n"),
        ("        obj.pose.pose       = decl.nominal;\n",
         "        obj.nominal         = decl.nominal;\n"
         "        setFieldPose(obj, decl.nominal);\n"),
        ("                obj.pose.pose = decl->nominal;\n",
         "                setFieldPose(obj, decl->nominal);\n"),
        ("            obj.pose.pose       = compose(robot.field_from_odom, obj.T_odom_object);\n",
         "            setFieldPose(obj, compose(robot.field_from_odom, obj.T_odom_object));\n"),
        ("        obj.pose.pose            = compose(robot.field_from_odom, obj.T_odom_object);\n",
         "        setFieldPose(obj, compose(robot.field_from_odom, obj.T_odom_object));\n"),
    ])

    # ---- target resolution: policy none through the helper
    edit(root, "src/impl/target_resolution/configured_targets.cpp", [
        ("#include \"resources/resource_store.h\"\n",
         "#include \"resources/resource_store.h\"\n#include \"state/landmark_error.h\"\n"),
        ("// Policy none: the current committed estimate, which is the nominal map\n"
         "// pose or a previously committed observation, never in-flight evidence.\n"
         "Pose2D ConfiguredTargetResolution::estimateTargetPose(\n"
         "    const TargetDecl& decl, const TargetResolutionInput& in) const {\n"
         "    Pose2D T_field_landmark;\n"
         "    if (const auto* lm = field_->find(decl.landmark)) {\n"
         "        T_field_landmark = lm->nominal;\n"
         "    }\n"
         "    const auto it = in.field.objects.find(decl.landmark);\n"
         "    if (it != in.field.objects.end() && it->second.valid) {\n"
         "        T_field_landmark = it->second.pose.pose;\n"
         "    }\n"
         "    const auto T_odom_landmark =\n"
         "        compose(inverse(in.robot.field_from_odom), T_field_landmark);\n"
         "    return decl.resolveFromLandmark(T_odom_landmark);\n"
         "}\n",
         "// Policy none: the nominal target moved with the landmark's committed\n"
         "// error (zero while unobserved), never in-flight evidence.\n"
         "Pose2D ConfiguredTargetResolution::estimateTargetPose(\n"
         "    const TargetDecl& decl, const TargetResolutionInput& in) const {\n"
         "    Pose2D        landmark_nominal;\n"
         "    LandmarkError error;\n"
         "    if (const auto* lm = field_->find(decl.landmark)) {\n"
         "        landmark_nominal = lm->nominal;\n"
         "    }\n"
         "    const auto it = in.field.objects.find(decl.landmark);\n"
         "    if (it != in.field.objects.end() && it->second.valid) {\n"
         "        // an estimate without a map nominal is its own nominal\n"
         "        landmark_nominal = it->second.nominal.value_or(it->second.pose.pose);\n"
         "        error            = it->second.error;\n"
         "    }\n"
         "    const Pose2D T_field_target =\n"
         "        applyLandmarkError(error, landmark_nominal, decl.resolveFromLandmark(landmark_nominal));\n"
         "    return compose(inverse(in.robot.field_from_odom), T_field_target);\n"
         "}\n"),
    ])

    # ---- inspection: nominal and error from the field state
    edit(root, "src/inspection/inspection_document.cpp", [
        ("    // nominal poses come from the configured maps, for heading error\n"
         "    std::map<std::string, Pose2D> nominal;\n"
         "    for (const ResourceStore::Record& r : system.resources().records()) {\n"
         "        std::string err;\n"
         "        auto        map = r.value.require<const FieldMap>(err);\n"
         "        if (map == nullptr) {\n"
         "            continue;\n"
         "        }\n"
         "        for (const LandmarkDecl& lm : map->landmarks) {\n"
         "            nominal.emplace(lm.id.value, lm.nominal);\n"
         "        }\n"
         "    }\n", ""),
        ("            const auto n = nominal.find(kv->first.value);\n"
         "            if (n != nominal.end()) {\n"
         "                pose2(w, \"nominal\", n->second);\n"
         "                w.field(\"heading_error_deg\",\n"
         "                        radToDeg(wrapAngle(o.pose.pose.heading_rad - n->second.heading_rad)));\n"
         "                w.field(\"displacement_m\",\n"
         "                        std::hypot(o.pose.pose.x_m - n->second.x_m,\n"
         "                                   o.pose.pose.y_m - n->second.y_m));\n",
         "            if (o.nominal) {\n"
         "                pose2(w, \"nominal\", *o.nominal);\n"
         "                w.field(\"heading_error_deg\", radToDeg(o.error.dtheta));\n"
         "                w.field(\"displacement_m\", std::hypot(o.error.dx, o.error.dy));\n"),
    ])

    # ---- tag traces out of the generic evidence header
    edit(root, "src/payloads/field_object_evidence.h", [
        ("// by intent belongs to target resolution.\n"
         "//\n"
         "// The trace set is the same decision made visible: every decoded tag with\n"
         "// its candidates, the accepted mount or the reason it was rejected, for\n"
         "// inspection. It never feeds estimation.\n",
         "// by intent belongs to target resolution.\n"),
        ("#include \"math/transforms.h\"\n#include \"state/pose_history.h\"\n",
         "#include \"math/transforms.h\"\n"),
        ("constexpr const char* kTagAssociationTraceSet = \"association.tag_association_trace_set\";\n",
         ""),
        ("\nstruct TagAssociationCandidate {\n"
         "    FieldObjectId object;\n"
         "    std::string   mount;\n"
         "    Pose2D        implied;   // T_odom_object\n"
         "    double        translation_error_m = 0.0;\n"
         "    double        heading_error_rad   = 0.0;\n"
         "    double        score               = 0.0;\n"
         "};\n", ""),
        ("\nstruct TagAssociationTrace {\n", "\nstruct REMOVED_TRACE {\n"),
    ])
    # drop everything from the placeholder to the closing namespace
    path = os.path.join(root, "src/payloads/field_object_evidence.h")
    with open(path, encoding="utf-8", newline="") as f:
        text = f.read()
    start = text.index("\nstruct REMOVED_TRACE {\n")
    end = text.index("\n} // namespace navigatr")
    text = text[:start] + "\n" + text[end:]
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(text)

    edit(root, "src/impl/world_estimation/tag_mount_association.cpp", [
        ("#include \"payloads/field_object_evidence.h\"\n",
         "#include \"impl/world_estimation/tag_association_trace.h\"\n"
         "#include \"payloads/field_object_evidence.h\"\n"),
    ])
    edit(root, "src/runtime/inspection_state.h", [
        ("#include \"core/time.h\"\n#include \"payloads/camera_frames.h\"\n"
         "#include \"payloads/field_object_evidence.h\"\n",
         "#include \"core/time.h\"\n#include \"impl/world_estimation/tag_association_trace.h\"\n"
         "#include \"payloads/camera_frames.h\"\n"),
    ])
    edit(root, "tests/capture_transform_gtest.cpp", [
        ("#include \"payloads/field_object_evidence.h\"\n",
         "#include \"impl/world_estimation/tag_association_trace.h\"\n"
         "#include \"payloads/field_object_evidence.h\"\n"),
    ])
    for rel in ["tests/synthetic_rig_gtest.cpp", "tests/vision_targets_gtest.cpp"]:
        edit(root, rel, [
            ("#include \"payloads/field_object_evidence.h\"\n",
             "#include \"impl/world_estimation/tag_association_trace.h\"\n"),
        ])

    # ---- tests
    edit(root, "tests/CMakeLists.txt", [
        ("    world_estimation_gtest.cpp\n",
         "    world_estimation_gtest.cpp\n    landmark_error_gtest.cpp\n"),
    ])
    with open(os.path.join(PROTO, "tests/world_estimation_add.cpp"), encoding="utf-8") as f:
        we_add = f.read()
    edit(root, "tests/world_estimation_gtest.cpp", [
        ("TEST(LandmarkEstimator, StrictSchema) {\n", we_add + "TEST(LandmarkEstimator, StrictSchema) {\n"),
    ])
    with open(os.path.join(PROTO, "tests/vision_targets_add.cpp"), encoding="utf-8") as f:
        vt_add = f.read()
    edit(root, "tests/vision_targets_gtest.cpp", [
        ("TEST(VisionTargets, OdometryEpochResetCancelsLatchedTarget) {\n",
         vt_add + "TEST(VisionTargets, OdometryEpochResetCancelsLatchedTarget) {\n"),
    ])
    with open(os.path.join(PROTO, "tests/inspection_document_add.cpp"), encoding="utf-8") as f:
        id_add = f.read()
    edit(root, "tests/inspection_document_gtest.cpp", [
        ("#include \"core/host_clock.h\"\n",
         "#include \"config/field_map.h\"\n#include \"core/host_clock.h\"\n"),
        ("#include \"inspection/json_writer.h\"\n",
         "#include \"inspection/json_writer.h\"\n#include \"math/angles.h\"\n"),
        ("TEST(InspectionDocuments, StateHistoryAndDiagSplitTheSnapshot) {\n",
         id_add + "TEST(InspectionDocuments, StateHistoryAndDiagSplitTheSnapshot) {\n"),
    ])


if __name__ == "__main__":
    main()
