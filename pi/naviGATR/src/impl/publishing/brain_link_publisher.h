// brain_link_publisher.h
// Answers the brain link request the brain_link commands slot processed this
// cycle, once, through the link's windowed write. No request, no output.
// The reply is built from this cycle's robot state, command state and
// health. Everything wire shaped is owned here and in translaGATR/: fixed point
// mm and centidegrees and the reply bodies. Health bits come only from
// configured references; an unreferenced bit stays clear.
//
//   <Publishing type="brain_link">
//       <Serial resource_id="brain_uart"/>       the brain_link commands resource
//       <Health fresh_ms="150">                  optional
//           <Encoder sensor_id="tracking_encoder_a"/>
//           <Gyro sensor_id="robot_imu"/>
//           <BiasCal function_id="tracking_motion"/>
//       </Health>
//       <Field resource_id="override_field"      optional, a field_map resource
//              estimate_period_ms="200"/>
//       <Pico resource_id="pico_telemetry"/>     optional, the Pico link health
//   </Publishing>
//
// With a Brain profile host (a Brain-profiled Localization) the health
// references follow the running profile: Health takes fresh_ms only, the
// encoders are the profile's, the gyro bit is the profile IMU source (the
// Pico IMU channel or the Brain bench mailbox), BiasCal and the calibration
// state follow the model that owns the IMU bias. Without a host they come
// from Health's children, and calibration from BiasCal. The calibration
// state is that model's stationary window calibration (none, collecting,
// done, waiting for stillness, waiting for data, failed).
//
// Health bits 4..6 come from the Pico link: PicoLink while its frames are
// fresh, ImuInitializing while the Pico reports its IMU initializing,
// aligning or retrying, ImuFailed once its quick attempts are used up.
// Bit 7 Stationary: some observation function's stationary window
// qualified and nothing moved since.
//
// The state block carries the robot, the profile status, the calibration
// state and, with a Field, the map_id and the newest estimate_id. The field documents are built by
// field_documents.h: the map once at build (the field needs a revision, a
// Boundary and a wire_id on every Landmark), an estimate snapshot in every
// reporting cycle whose content changed at most once per
// estimate_period_ms, the last three retained for chunked READ_DOC. Without
// a Field, map_id and estimate_id are 0 and READ_DOC is Unavailable. Every
// cycle, request or not, the state block it would answer goes out in
// PublishingOutput::brain_state for inspection.
//
// Instrumentation: each reply goes to the link's LinkMonitor as attempted
// and accepted bytes (a refused write accepted nothing) with a decoded
// summary, and completes the request record the commands slot staged.

#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "translaGATR/frame_codec.h"
#include "contracts/brain_profile.h"
#include "contracts/publishing.h"
#include "impl/publishing/field_documents.h"
#include "resources/pico_control.h"
#include "resources/serial_link.h"

namespace navigatr
{

class DiagnosticsHub;
class LinkMonitor;

class BrainLinkPublisher : public Publishing
{
public:
    static std::unique_ptr<Publishing> create(const ConfigNode& node,
                                              SlotInitializationContext& context,
                                              std::string& err);

    PublishingOutput run(const PublishingInput& in) override;

    void reset() override;

    // The served field documents, null without a Field.
    const FieldDocuments* fieldDocuments() const { return documents_.get(); }

private:
    void noteReply(const translagatr::BrainReply& reply, const uint8_t* frame, uint16_t len,
                   const SerialWriteResult& written);
    bool              sensorFresh(const SensorMap& results, const SensorId& id,
                                  MonotonicTime now) const;
    uint8_t           calibration(const PublishingInput& in, const ProfileBinding* profile) const;
    translagatr::BrainState state(const PublishingInput& in) const;

    std::shared_ptr<SerialLink> link_;
    std::string                 diagnostics_id_;

    std::vector<SensorId> encoder_health_;      // bit needs every one fresh
    SensorId              gyro_health_;         // empty = bit stays clear
    std::string           bias_cal_function_;   // ready state latches the bit
    long                  fresh_ms_ = 150;

    std::unique_ptr<FieldDocuments> documents_;
    BrainProfileHost*               profile_host_ = nullptr;
    std::shared_ptr<PicoControl>    pico_;

    DiagnosticsHub*              hub_       = nullptr;   // null outside a System
    uint16_t                     source_id_ = 0;
    std::shared_ptr<LinkMonitor> monitor_;   // shared with the brain_link commands

    bool bias_cal_seen_ = false;
};

} // namespace navigatr
