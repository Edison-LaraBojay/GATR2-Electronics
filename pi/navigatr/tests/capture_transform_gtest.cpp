// capture_transform_gtest.cpp
// The capture-time chain T_odom_tag = T_odom_robot(t) * T_robot_camera *
// T_camera_tag against ground truth: a camera mounted off the fixed robot
// origin, an in-place turn interpolated from history at the exposure
// instant, a delayed frame, measured roll and pitch versus the assumed
// level fallback, and the explicit refusals when history cannot place the
// exposure.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include "impl/association/tag_mount_association.h"
#include "math/angles.h"
#include "math/quaternion.h"
#include "math/se3.h"
#include "payloads/field_object_evidence.h"
#include "payloads/tag_observations.h"
#include "resources/resource_store.h"
#include "runtime/register_all.h"
#include "state/pose_history.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{

// One goal with one mount facing -x, a camera 18 cm ahead, 5 cm left and 22 cm
// up, pitched down 8 degrees and yawed 5 degrees left.
const char* kResources = R"(
<Resources>
    <Resource id="field" type="field_map">
        <Landmark id="goal">
            <NominalPose calibration_status="verified" x_m="2.0" y_m="1.0" heading_deg="0"/>
            <TagMount instance_id="goal_west" calibration_status="verified"
                      family="tag36h11" observed_id="7" detection_size_m="0.06">
                <PoseOfTagSurfaceInLandmark x_m="-0.1" y_m="0" z_m="0.2"
                    roll_deg="0" pitch_deg="0" yaw_deg="180"/>
            </TagMount>
        </Landmark>
    </Resource>
    <Resource id="frames" type="robot_frame_map">
        <Frame id="cam" parent_frame_id="robot_body" calibration_status="verified">
            <PoseOfChildInParent x_m="0.18" y_m="0.05" z_m="0.22"
                roll_deg="0" pitch_deg="8" yaw_deg="5"/>
        </Frame>
    </Resource>
</Resources>)";

const char* kAssociation = R"(
<Association type="tag_mount_association">
    <Observations observation_id="tags"/>
    <FieldMap resource_id="field"/>
    <RobotFrames resource_id="frames"/>
    <Attitude policy="assume_level"/>
    <Gates max_translation_error_m="0.5" max_heading_error_deg="30"
           ambiguity_margin_m="0.15" max_range_m="4.0"
           min_decision_margin="10" max_hamming="0"
           min_facing_cos="0.1" min_projected_size_px="8"/>
    <Output association_id="evidence"/>
    <Trace association_id="trace"/>
</Association>)";

struct Fixture {
    FunctionRegistry             functions;
    std::vector<std::string>     warnings;
    ResourceStore                store;
    tinyxml2::XMLDocument        assoc_doc;
    std::unique_ptr<Association> assoc;
    std::shared_ptr<const FieldMap>      field;
    std::shared_ptr<const RobotFrameMap> frames;
    Transform3                           T_robot_camera;
    Transform3                           T_landmark_tag;
    Pose2D                               landmark_truth{2.0, 1.0, 0.0};

    Fixture() {
        registerAll(functions);
        tinyxml2::XMLDocument doc;
        EXPECT_EQ(doc.Parse(kResources), tinyxml2::XML_SUCCESS);
        ResourceStoreBuilder builder(functions, &warnings);
        std::string          err;
        bool                 ok = true;
        ConfigNode{doc.RootElement()}.forEach("Resource", [&](const ConfigNode& r) {
            if (ok) {
                ok = builder.index(r, err);
            }
        });
        EXPECT_TRUE(ok && builder.buildAll(err)) << err;
        store  = builder.take();
        field  = store.require<const FieldMap>(ResourceId{"field"}, err);
        frames = store.require<const RobotFrameMap>(ResourceId{"frames"}, err);
        EXPECT_NE(field, nullptr) << err;
        EXPECT_NE(frames, nullptr) << err;
        T_robot_camera = *frames->find(FrameId{"cam"});
        T_landmark_tag = field->landmarks[0].mounts[0].T_landmark_tag_surface;

        SlotInitializationContext context;
        context.resources = &store;
        context.functions = &functions;
        context.observations.push_back(ObservationOutputDecl{
            ObservationId{"tags"},
            PayloadDescriptor::of<TagObservationSet>(payload_names::kTagObservationSet)});
        EXPECT_EQ(assoc_doc.Parse(kAssociation), tinyxml2::XML_SUCCESS);
        assoc = TagMountAssociation::create(ConfigNode{assoc_doc.RootElement()}, context, err);
        EXPECT_NE(assoc, nullptr) << err;
    }

    // Body pose in the odometry frame at a truth instant.
    static Transform3 bodyTransform(const Pose2D& pose, double roll, double pitch) {
        Transform3 T;
        T.R   = rotationFromEuler(roll, pitch, pose.heading_rad);
        T.x_m = pose.x_m;
        T.y_m = pose.y_m;
        T.z_m = 0.0;
        return T;
    }

    // The exact tag pose a perfect detector would report for this truth.
    ObservationMap observe(const Pose2D& truth_pose, double roll, double pitch,
                           MonotonicTime exposure, uint32_t sequence = 1) const {
        const Transform3 T_odom_camera =
            compose(bodyTransform(truth_pose, roll, pitch), T_robot_camera);
        const Transform3 T_odom_tag =
            compose(transform3FromPlanar(landmark_truth), T_landmark_tag);
        TagObservation tag;
        tag.family          = "tag36h11";
        tag.observed_id     = 7;
        tag.has_pose        = true;
        tag.T_camera_tag    = compose(inverse(T_odom_camera), T_odom_tag);
        tag.hamming         = 0;
        tag.decision_margin = 60.0;
        TagObservationSet set;
        set.camera         = SensorId{"front_camera"};
        set.camera_frame   = FrameId{"cam"};
        set.frame_sequence = sequence;
        set.exposureAt     = exposure;
        set.tags.push_back(tag);
        ObservationMap map;
        ObservationRecord record;
        record.measuredAt = exposure;
        record.payload    = TypedPayload::store(set, payload_names::kTagObservationSet);
        map.emplace(ObservationId{"tags"}, std::move(record));
        return map;
    }

    RobotState robotAt(const PoseHistory& history) const {
        RobotState robot;
        robot.valid          = true;
        robot.odom_pose      = history.newest().odom_pose;
        robot.odometry_epoch = history.newest().odometry_epoch;
        return robot;
    }

    AssociationOutput run(const ObservationMap& obs, const PoseHistory& history,
                          MonotonicTime now, uint64_t epoch_override = 0) {
        RobotState robot = robotAt(history);
        if (epoch_override != 0) {
            robot.odometry_epoch = epoch_override;
        }
        FieldState field_state;
        return assoc->run({obs, robot, history, field_state, now});
    }

    static const FieldObjectPoseEvidenceSet* evidence(const AssociationOutput& out) {
        const auto it = out.associations.find(AssociationId{"evidence"});
        return it == out.associations.end()
                   ? nullptr
                   : it->second.payload.get<FieldObjectPoseEvidenceSet>();
    }

    static const TagAssociationTraceSet* trace(const AssociationOutput& out) {
        const auto it = out.associations.find(AssociationId{"trace"});
        return it == out.associations.end() ? nullptr
                                            : it->second.payload.get<TagAssociationTraceSet>();
    }
};

// In-place turn about the fixed origin: heading 0 to 90 degrees over one
// second, 10 ms samples, optional constant tilt recorded as measured.
PoseHistory turnHistory(bool with_attitude, double roll = 0.0, double pitch = 0.0) {
    PoseHistory history;
    for (int i = 0; i <= 100; ++i) {
        PoseHistoryEntry e;
        e.at             = hostTime(1000 + i * 10);
        e.odom_pose      = Pose2D{1.0, 1.0, degToRad(0.9 * i)};
        e.odometry_epoch = 1;
        if (with_attitude) {
            e.attitude.valid            = true;
            e.attitude.reference        = "odometry";
            e.attitude.q_reference_body = quaternionFromEuler(roll, pitch, e.odom_pose.heading_rad);
            e.attitude.measuredAt       = e.at;
        }
        history.append(e);
    }
    return history;
}

} // namespace

TEST(CaptureTransform, InPlaceTurnUsesTheInterpolatedPoseAtExposure) {
    Fixture     f;
    PoseHistory history = turnHistory(false);

    // exposure between two samples: heading 45.45 degrees, origin unmoved
    const MonotonicTime exposure = hostTime(1505);
    const Pose2D        truth{1.0, 1.0, degToRad(45.45)};
    const AssociationOutput out = f.run(f.observe(truth, 0.0, 0.0, exposure), history,
                                        hostTime(2100));
    const FieldObjectPoseEvidenceSet* ev = Fixture::evidence(out);
    ASSERT_NE(ev, nullptr);
    ASSERT_EQ(ev->entries.size(), 1u);
    EXPECT_EQ(ev->entries[0].object.value, "goal");
    EXPECT_EQ(ev->entries[0].feature_instance, "goal_west");
    EXPECT_NEAR(ev->entries[0].T_frame_object.x_m, 2.0, 1e-6);
    EXPECT_NEAR(ev->entries[0].T_frame_object.y_m, 1.0, 1e-6);
    EXPECT_NEAR(ev->entries[0].T_frame_object.heading_rad, 0.0, 1e-6);
    EXPECT_TRUE(ev->entries[0].attitude_assumed);   // no tilt was recorded
    EXPECT_EQ(ev->entries[0].odometry_epoch, 1u);

    // the camera itself moved during the turn while the origin did not:
    // the same tag is seen from two different camera positions
    const Transform3 cam0 =
        compose(Fixture::bodyTransform(Pose2D{1.0, 1.0, 0.0}, 0.0, 0.0), f.T_robot_camera);
    const Transform3 cam1 = compose(Fixture::bodyTransform(truth, 0.0, 0.0), f.T_robot_camera);
    EXPECT_GT(std::hypot(cam1.x_m - cam0.x_m, cam1.y_m - cam0.y_m), 0.1);

    // using the newest pose instead of the exposure-time pose would be
    // visibly wrong: re-run with a history holding only the final heading
    PoseHistory wrong;
    PoseHistoryEntry e;
    e.at             = exposure;
    e.odom_pose      = Pose2D{1.0, 1.0, degToRad(90.0)};
    e.odometry_epoch = 1;
    wrong.append(e);
    const AssociationOutput out_wrong =
        f.run(f.observe(truth, 0.0, 0.0, exposure), wrong, hostTime(2100));
    const FieldObjectPoseEvidenceSet* ev_wrong = Fixture::evidence(out_wrong);
    if (ev_wrong != nullptr && !ev_wrong->entries.empty()) {
        EXPECT_GT(std::hypot(ev_wrong->entries[0].T_frame_object.x_m - 2.0,
                             ev_wrong->entries[0].T_frame_object.y_m - 1.0),
                  0.3);
    }
}

TEST(CaptureTransform, DelayedFrameStillResolvesAtItsExposureTime) {
    Fixture     f;
    PoseHistory history = turnHistory(false);

    // the frame arrives 480 ms after exposure, the robot kept turning
    const MonotonicTime exposure = hostTime(1200);
    const Pose2D        truth{1.0, 1.0, degToRad(18.0)};
    const AssociationOutput out =
        f.run(f.observe(truth, 0.0, 0.0, exposure), history, hostTime(2000));
    const FieldObjectPoseEvidenceSet* ev = Fixture::evidence(out);
    ASSERT_NE(ev, nullptr);
    ASSERT_EQ(ev->entries.size(), 1u);
    EXPECT_NEAR(ev->entries[0].T_frame_object.x_m, 2.0, 1e-6);
    EXPECT_NEAR(ev->entries[0].T_frame_object.y_m, 1.0, 1e-6);
    EXPECT_EQ(ev->entries[0].measuredAt.ms, 1200);   // exposure, not receipt
}

TEST(CaptureTransform, MeasuredTiltIsAppliedAndTheLevelFallbackIsLabeled) {
    Fixture      f;
    // a large tilt: the planar projection of a level assumption hides small
    // tilts (the error is mostly vertical), 15 degrees makes it visible
    const double roll  = degToRad(4.0);
    const double pitch = degToRad(-15.0);
    const MonotonicTime exposure = hostTime(1505);
    const Pose2D        truth{1.0, 1.0, degToRad(45.45)};
    const ObservationMap obs = f.observe(truth, roll, pitch, exposure);

    // with the tilt in history the chain is exact and marked measured
    PoseHistory              tilted = turnHistory(true, roll, pitch);
    const AssociationOutput  out    = f.run(obs, tilted, hostTime(2100));
    const FieldObjectPoseEvidenceSet* ev = Fixture::evidence(out);
    ASSERT_NE(ev, nullptr);
    ASSERT_EQ(ev->entries.size(), 1u);
    EXPECT_FALSE(ev->entries[0].attitude_assumed);
    EXPECT_NEAR(ev->entries[0].T_frame_object.x_m, 2.0, 2e-3);
    EXPECT_NEAR(ev->entries[0].T_frame_object.y_m, 1.0, 2e-3);
    EXPECT_NEAR(ev->entries[0].T_frame_object.heading_rad, 0.0, 2e-3);

    // without it the level assumption is used, labeled, and visibly off
    PoseHistory             level     = turnHistory(false);
    const AssociationOutput out_level = f.run(obs, level, hostTime(2100));
    const FieldObjectPoseEvidenceSet* ev_level = Fixture::evidence(out_level);
    ASSERT_NE(ev_level, nullptr);
    ASSERT_EQ(ev_level->entries.size(), 1u);
    EXPECT_TRUE(ev_level->entries[0].attitude_assumed);
    EXPECT_GT(std::hypot(ev_level->entries[0].T_frame_object.x_m - 2.0,
                         ev_level->entries[0].T_frame_object.y_m - 1.0),
              0.02);
}

TEST(CaptureTransform, HistoryRefusalsAreExplicitInTheTrace) {
    Fixture     f;
    PoseHistory history = turnHistory(false);
    const Pose2D truth{1.0, 1.0, 0.0};

    // exposure newer than the newest pose: pending, no evidence
    AssociationOutput out =
        f.run(f.observe(truth, 0.0, 0.0, hostTime(2500)), history, hostTime(2600));
    EXPECT_EQ(Fixture::evidence(out), nullptr);
    const TagAssociationTraceSet* tr = Fixture::trace(out);
    ASSERT_NE(tr, nullptr);
    EXPECT_NE(tr->frame_note.find("pending"), std::string::npos);

    // exposure older than the ring: expired
    out = f.run(f.observe(truth, 0.0, 0.0, hostTime(100)), history, hostTime(2600));
    EXPECT_EQ(Fixture::evidence(out), nullptr);
    tr = Fixture::trace(out);
    ASSERT_NE(tr, nullptr);
    EXPECT_NE(tr->frame_note.find("expired"), std::string::npos);

    // the robot moved on to a new odometry epoch: the history answer is
    // from the old frame and is refused
    out = f.run(f.observe(truth, 0.0, 0.0, hostTime(1505)), history, hostTime(2600), 2);
    EXPECT_EQ(Fixture::evidence(out), nullptr);
    tr = Fixture::trace(out);
    ASSERT_NE(tr, nullptr);
    EXPECT_NE(tr->frame_note.find("epoch"), std::string::npos);

    // a 2D decode never becomes evidence, and says why
    ObservationMap obs = f.observe(truth, 0.0, 0.0, hostTime(1505));
    {
        TagObservationSet set =
            *obs.at(ObservationId{"tags"}).payload.get<TagObservationSet>();
        set.tags[0].has_pose = false;
        obs.at(ObservationId{"tags"}).payload =
            TypedPayload::store(set, payload_names::kTagObservationSet);
    }
    out = f.run(obs, history, hostTime(2600));
    EXPECT_EQ(Fixture::evidence(out), nullptr);
    tr = Fixture::trace(out);
    ASSERT_NE(tr, nullptr);
    ASSERT_EQ(tr->tags.size(), 1u);
    EXPECT_FALSE(tr->tags[0].accepted);
    EXPECT_NE(tr->tags[0].rejection.find("no metric pose"), std::string::npos);
}
