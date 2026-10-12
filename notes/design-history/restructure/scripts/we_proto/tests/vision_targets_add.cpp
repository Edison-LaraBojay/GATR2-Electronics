TEST(VisionTargets, PolicyNoneMovesTheNominalTargetWithTheLandmarkError) {
    // a displaced, turned goal committed under a non-identity anchor
    Fixture f;
    f.robot->odom            = Pose2D{0.9, 1.7832, 0.0};
    f.robot->field_from_odom = Pose2D{0.1, -0.05, 0.1};
    const Pose2D displaced{kCenterGoal.x_m + 0.05, kCenterGoal.y_m - 0.03, 0.08};
    f.pushFrame({f.detectionFor(displaced, kCenterMount, 0)});
    f.stepOnce();
    ASSERT_EQ(f.system->field().objects.at(FieldObjectId{"center_goal"}).source,
              EstimateSource::kObserved);

    f.select(3);   // policy none
    f.stepOnce();
    ASSERT_EQ(f.system->target().status, TargetStatus::kLockedNominal);
    const Pose2D latched   = f.system->target().T_odom_robot_target;
    const Pose2D goal_pose = f.system->field().objects.at(FieldObjectId{"center_goal"}).pose.pose;

    std::string err;
    const auto  targets =
        f.system->resources().require<const TargetSet>(ResourceId{"targets"}, err);
    ASSERT_NE(targets, nullptr) << err;
    const TargetDecl* decl = targets->findByWireId(3);
    ASSERT_NE(decl, nullptr);
    const Pose2D T_odom_field = inverse(f.robot->field_from_odom);

    // the full-pose composition this replaced
    const Pose2D expected = decl->resolveFromLandmark(compose(T_odom_field, goal_pose));
    EXPECT_NEAR(latched.x_m, expected.x_m, 1e-9);
    EXPECT_NEAR(latched.y_m, expected.y_m, 1e-9);
    EXPECT_NEAR(wrapAngle(latched.heading_rad - expected.heading_rad), 0.0, 1e-9);

    // and not the nominal target
    const Pose2D nominal = decl->resolveFromLandmark(compose(T_odom_field, kCenterGoal));
    EXPECT_GT(std::hypot(latched.x_m - nominal.x_m, latched.y_m - nominal.y_m), 0.03);
}

