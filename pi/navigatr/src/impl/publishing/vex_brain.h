// vex_brain.h
// Encodes the VEX brain pose packet every cycle while streaming is on, even
// when the pose is invalid, so the brain always sees health. Everything
// wire shaped is owned here: fixed point mm and centidegree conversion,
// packet layout via common/, and the mapping between configured world object
// ids and the brain's wire object ids. Health bits come only from configured
// references; an unreferenced bit stays clear.
//
//   <Publishing type="publishing/vex_brain">
//       <Serial resource_id="brain_uart"/>
//       <Health fresh_ms="150">
//           <Encoder sensor_id="tracking_encoder_a"/>
//           <Encoder sensor_id="tracking_encoder_b"/>
//           <Gyro sensor_id="robot_imu"/>
//           <BiasCal function_id="tracking_motion"/>
//       </Health>
//       <FieldObject object_id="center_goal" wire_id="1"/>
//       <Landmarks association_id="landmarks"/>   optional wire entries
//   </Publishing>

#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "contracts/publishing.h"
#include "resources/serial_link.h"

namespace navigatr
{

class VexBrainPublisher : public Publishing
{
public:
    static std::unique_ptr<Publishing> create(const ConfigNode& node,
                                              SlotInitializationContext& context,
                                              std::string& err);

    PublishingOutput run(const PublishingInput& in) override;

    void reset() override;

private:
    bool sensorFresh(const SensorMap& results, const SensorId& id,
                     MonotonicTime now) const;

    std::shared_ptr<SerialLink> link_;

    std::vector<SensorId> encoder_health_;   // enc bit needs every one fresh
    SensorId              gyro_health_;      // empty = bit stays clear
    std::string           bias_cal_function_;   // ready state latches the bias bit
    long                  fresh_ms_ = 150;

    struct WireObject {
        FieldObjectId object;
        uint8_t       wire_id = 0;
    };
    std::vector<WireObject> wire_objects_;

    AssociationId landmarks_ref_;   // empty = no wire landmark entries

    bool    bias_cal_seen_ = false;
    uint8_t seq_           = 0;
};

} // namespace navigatr
