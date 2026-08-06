// tag_detectors.h
// Fiducial detector resources. apriltag_detector is the upstream
// AprilRobotics detector behind the TagDetector contract; its adapter (and
// the vendored library) land with camera bring-up. Until then the type is
// registered so configurations validate their schema, and selecting it
// fails loudly instead of silently detecting nothing.
//
//   <Resource id="tag_detector" type="apriltag_detector">
//       <Family name="tag36h11" detection_size_m="0.060"/>
//   </Resource>
//
// detection_size_m is the physical edge length of the detector's four
// pose-estimation corners, not the outer sticker dimension.

#pragma once
#include <string>

#include "config/config_node.h"
#include "resources/resource_instance.h"
#include "resources/resource_map.h"

namespace navigatr
{

ResourceInstance make_apriltag_detector(const ConfigNode&              node,
                                        ResourceInitializationContext& context,
                                        std::string&                   err);

} // namespace navigatr
