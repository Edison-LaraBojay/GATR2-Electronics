// brain_link_publisher.h
// Answers the brain link request the brain_link commands slot processed this
// cycle, once, through the link's windowed write. No request, no output.
// The reply is built from this cycle's robot state, the newest field
// snapshot and health. Everything wire shaped is owned here and in common/:
// fixed point mm and centidegrees, reply bodies, and the mapping between
// configured field object ids and landmark wire ids. Health bits come only
// from configured references; an unreferenced bit stays clear.
//
//   <Publishing type="brain_link">
//       <Serial resource_id="brain_uart"/>       the brain_link commands resource
//       <Health fresh_ms="150">                  optional
//           <Encoder sensor_id="tracking_encoder_a"/>
//           <Gyro sensor_id="robot_imu"/>
//           <BiasCal function_id="tracking_motion"/>
//       </Health>
//       <FieldObject object_id="center_goal" wire_id="1"/>   repeatable, 1..255
//   </Publishing>
//
// Landmark fields are the physical landmark pose under the robot's field
// anchor, never a resolved destination: an observed estimate from the
// robot's odometry epoch (source observed, with its age), else the map
// nominal (source nominal), else source none. World estimation noop always
// reports none and answers a selection with kResultLandmarkUnsupported.

#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "common/frame_codec.h"
#include "contracts/publishing.h"
#include "resources/serial_link.h"

namespace navigatr
{

class BrainLinkPublisher : public Publishing
{
public:
    static std::unique_ptr<Publishing> create(const ConfigNode& node,
                                              SlotInitializationContext& context,
                                              std::string& err);

    PublishingOutput run(const PublishingInput& in) override;

    void reset() override;

private:
    struct WireObject {
        FieldObjectId object;
        uint8_t       wire_id = 0;
    };

    bool              sensorFresh(const SensorMap& results, const SensorId& id,
                                  MonotonicTime now) const;
    const WireObject* findWire(uint8_t wire_id) const;
    uint8_t           selectResult(const BrainReplyContext& ctx) const;
    gatr2::BrainState state(const PublishingInput& in) const;

    std::shared_ptr<SerialLink> link_;
    std::string                 diagnostics_id_;
    bool                        world_noop_ = false;

    std::vector<SensorId> encoder_health_;      // bit needs every one fresh
    SensorId              gyro_health_;         // empty = bit stays clear
    std::string           bias_cal_function_;   // ready state latches the bit
    long                  fresh_ms_ = 150;

    std::vector<WireObject> wire_objects_;

    bool bias_cal_seen_ = false;
};

} // namespace navigatr
