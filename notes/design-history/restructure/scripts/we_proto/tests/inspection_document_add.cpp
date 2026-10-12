TEST(InspectionDocuments, FieldObjectErrorsAreThePoseMinusNominal) {
    FunctionRegistry functions;
    registerAll(functions);
    std::string             err;
    std::unique_ptr<System> system = System::buildFromString(kRig, functions, err);
    ASSERT_NE(system, nullptr) << err;
    int64_t now = 1;
    for (int i = 0; i < 180; ++i) {
        now += 10;
        system->step(hostTime(now));
    }
    const FieldObjectState goal = system->field().objects.at(FieldObjectId{"goal"});
    ASSERT_EQ(goal.source, EstimateSource::kObserved);
    const auto map = system->resources().require<const FieldMap>(ResourceId{"field"}, err);
    ASSERT_NE(map, nullptr) << err;
    const Pose2D nominal = map->find(FieldObjectId{"goal"})->nominal;

    // the fields as written before the error existed
    JsonWriter w;
    w.beginObject();
    w.field("heading_error_deg",
            radToDeg(wrapAngle(goal.pose.pose.heading_rad - nominal.heading_rad)));
    w.field("displacement_m",
            std::hypot(goal.pose.pose.x_m - nominal.x_m, goal.pose.pose.y_m - nominal.y_m));
    w.endObject();
    const std::string legacy = w.str().substr(1, w.str().size() - 2);

    const std::string snap = snapshotDocument(*system, InspectionServiceStats{}, hostTime(now));
    const std::size_t at   = snap.find("\"field_objects\":[{\"id\":\"goal\"");
    ASSERT_NE(at, std::string::npos) << snap;
    const std::string object = balancedObject(snap, snap.find('{', at));
    EXPECT_NE(object.find(legacy), std::string::npos) << object << "\n" << legacy;
    EXPECT_EQ(object.find("\"heading_error_deg\":0,"), std::string::npos) << object;
}

