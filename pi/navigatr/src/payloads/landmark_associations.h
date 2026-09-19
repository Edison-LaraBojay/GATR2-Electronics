// payloads/landmark_associations.h
// Landmark observations resolved to configured landmark objects, SI units,
// field and robot frames. The optional Landmarks input of the VEX brain
// publisher accepts this payload from a configured association function.
// Wire ids are not part of this payload; the publisher maps object ids to
// its wire protocol.

#pragma once
#include <string>
#include <vector>

#include "core/ids.h"

namespace navigatr
{

namespace payload_names
{
constexpr const char* kLandmarkAssociationSet = "association.landmark_set";
} // namespace payload_names

struct LandmarkAssociationEntry {
    FieldObjectId landmark;   // matches a configured landmark object

    // live relative transform, robot frame
    double dx_m        = 0.0;
    double dy_m        = 0.0;
    double bearing_rad = 0.0;

    // the observation projected into the field frame
    double field_x_m = 0.0;
    double field_y_m = 0.0;

    double quality = 0.0;   // 0..1
};

struct LandmarkAssociationSet {
    std::vector<LandmarkAssociationEntry> entries;
};

} // namespace navigatr
