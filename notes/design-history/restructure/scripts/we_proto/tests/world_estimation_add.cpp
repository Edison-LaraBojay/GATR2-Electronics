TEST(LandmarkEstimator, SeededLandmarksCarryTheirNominalAndZeroError) {
    LandmarkFixture f;
    std::string     err;
    auto            estimator = f.make(kNever, err);
    ASSERT_TRUE(estimator.has_value()) << err;
    f.run(*estimator);

    const Pose2D nominal = f.map->find(FieldObjectId{"center_goal"})->nominal;
    const auto&  goal    = f.field.objects.at(FieldObjectId{"center_goal"});
    ASSERT_TRUE(goal.nominal.has_value());
    EXPECT_EQ(goal.nominal->x_m, nominal.x_m);
    EXPECT_EQ(goal.nominal->y_m, nominal.y_m);
    EXPECT_EQ(goal.nominal->heading_rad, nominal.heading_rad);
    EXPECT_EQ(goal.error.dx, 0.0);
    EXPECT_EQ(goal.error.dy, 0.0);
    EXPECT_EQ(goal.error.dtheta, 0.0);
}

TEST(LandmarkEstimator, ErrorIsDerivedWithTheFieldPoseUnderEveryAnchor) {
    LandmarkFixture f;
    std::string     err;
    auto            estimator = f.make(kAlways, err);
    ASSERT_TRUE(estimator.has_value()) << err;
    const Pose2D nominal = f.map->find(FieldObjectId{"center_goal"})->nominal;
    const auto   expectDerived = [&] {
        const FieldObjectState& goal     = f.field.objects.at(FieldObjectId{"center_goal"});
        const LandmarkError     expected = landmarkError(nominal, goal.pose.pose);
        EXPECT_EQ(goal.error.dx, expected.dx);
        EXPECT_EQ(goal.error.dy, expected.dy);
        EXPECT_EQ(goal.error.dtheta, expected.dtheta);
    };

    f.robot.field_from_odom = Pose2D{0.3, -0.2, 0.25};
    f.run(*estimator);
    const auto set = LandmarkFixture::one("center_goal", 1.6, 1.9, kPi / 2.0 - 0.3, 0.9);
    f.run(*estimator, &set);
    expectDerived();
    const LandmarkError moved = f.field.objects.at(FieldObjectId{"center_goal"}).error;
    EXPECT_GT(std::hypot(moved.dx, moved.dy), 0.1);

    // a re-anchor re-derives pose and error together
    f.robot.anchor_revision += 1;
    f.robot.field_from_odom = Pose2D{-0.4, 0.1, -1.0};
    f.run(*estimator);
    expectDerived();

    // a new odometry epoch is back to exactly as built
    f.robot.odometry_epoch += 1;
    f.run(*estimator);
    const LandmarkError reset = f.field.objects.at(FieldObjectId{"center_goal"}).error;
    EXPECT_EQ(reset.dx, 0.0);
    EXPECT_EQ(reset.dy, 0.0);
    EXPECT_EQ(reset.dtheta, 0.0);
}

