// pose_source.h
// CALLER ODOMETRY: where a robot program hands the demo its own pose.
// Replace currentPose() in src/pose_source.cpp with your odometry. Field
// frame, meters and radians: spectaGATR/docs/PROTOCOL.md.

#pragma once
#include "spectagatr/pose.h"

struct OdometryPose {
    spectagatr::Meters  x       = 0;
    spectagatr::Meters  y       = 0;
    spectagatr::Radians heading = 0; // CCW from field +x
};

// Called by the demo task every loop: quick, never blocking.
OdometryPose currentPose();

// True while currentPose() is the placeholder; the Brain screen says so.
bool currentPoseIsPlaceholder();
