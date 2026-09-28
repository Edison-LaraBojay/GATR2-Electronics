// brain_profile_builder.h
// Pi side of a Brain robot profile: the <BrainProfile> element that opts a
// configuration in, this Pi's capability checks, and the typed profile
// turned into Sensors and Localization subtrees from fixed Pi templates. The
// Brain never sends XML; every id is generated here, and the generated
// subtrees go through the ordinary factories and their checks.
//
//   <Localization>
//       <BrainProfile>
//           <Encoders resource_id="pico_telemetry" stale_after_ms="250">
//               <Port index="0" output_id="encoder_a"/>     one per wired port, 0..2
//           </Encoders>
//           <Imu port="0" resource_id="pico_telemetry" output_id="imu"
//                stale_after_ms="250"/>                     optional, the Pico IMU
//           <BrainImu resource_id="brain_imu"/>             optional, a brain_imu_bench
//           <Calibration bias_samples="20" window_ms="2000" max_gap_ms="250"
//                        still_travel_m="0.001" still_rate_dps="1" max_rate_dps="5"
//                        evidence_gap_ms="100" attempt_s="60"/>     optional
//           <Timing interval_tolerance_ms="20" max_pending_ms="500"
//                   sensor_loss_ms="250" on_sensor_loss="unplace"/>   optional
//           <Fusion max_wait_ms="100">                      three wheels with the Pico IMU
//               <MotionNoise translation_floor_m=".." translation_per_m=".."
//                            rotation_floor_rad=".." rotation_per_rad=".."
//                            rotation_per_m=".."/>
//               <HeadingNoise angle_random_walk_rad_per_sqrt_s=".." bias_rad_per_s=".."/>
//           </Fusion>
//           <History retention_s="5" capacity="1024" max_interpolation_gap_ms="100"
//                    attitude_gap_ms="100"/>                optional
//       </BrainProfile>
//   </Localization>
//
// A Brain-profiled Localization holds only BrainProfile: the Pi owns the
// devices, the wired ports and the model tuning; the profile owns geometry,
// topology, the IMU source and the footprint. Calibration holds the Pi
// defaults for the stationary window every IMU bias path and the stationary
// status use (stationary_window.h): bias_samples per source, window_ms of
// sample time, per-wheel still_travel_m, gyro still_rate_dps and
// max_rate_dps, evidence_gap_ms between samples, attempt_s before a
// calibration fails; max_gap_ms is the gyro integration gap. A profile's
// calibration_window_ms, still_rate_cdps and still_travel_um replace
// window_ms, still_rate_dps and still_travel_m when nonzero. Timing
// sensor_loss_ms is how long a used source may go without a sample before
// pose continuity is lost (spec 8.10, sensor_loss.h); on_sensor_loss
// unplace (default) unplaces the robot then, warn only logs it.
//
// Models per topology and IMU source; anything else is refused:
//   two wheel, pico           tracking_wheel_motion + HeadingConstraint,
//                             planar_motion_integrator
//   two wheel, brain_vex      brain_imu_planar_bench, planar_motion_integrator
//   two forward, pico         tracking_wheel_motion + HeadingConstraint +
//                             LateralMotion zero, planar_motion_integrator
//   two forward, brain_vex    brain_imu_parallel_bench, planar_motion_integrator
//   three wheel, none         tracking_wheel_motion, planar_motion_integrator
//   three wheel, pico         tracking_wheel_motion (no constraint) +
//                             imu_heading_increment, weighted_planar_fusion
//
// Corrections, each applied once: counts per revolution, gearing and
// polarity in the encoder sensor; radius, mounting, measuring angle and the
// travel scale in the observation model, whose geometric direction is always
// positive; the Pico IMU sign in its sensor.

#pragma once
#include <array>
#include <cstdint>
#include <string>

#include "common/link_documents.h"
#include "config/config_node.h"
#include "contracts/brain_profile.h"
#include "core/ids.h"
#include "resources/resource_store.h"
#include "runtime/resource_catalog.h"

namespace tinyxml2
{
class XMLDocument;
} // namespace tinyxml2

namespace navigatr
{

constexpr uint8_t kProfileEncoderPorts = 3;

// Generated sensor ids start with this; configured sensors may not.
constexpr const char* kProfileSensorPrefix = "profile_";

struct BrainProfileConfig {
    ResourceId encoder_resource;
    long       encoder_stale_after_ms = 250;
    std::array<OutputId, kProfileEncoderPorts> ports;   // empty = not wired

    bool       imu = false;   // a Pico IMU port is configured
    uint8_t    imu_port = 0;
    ResourceId imu_resource;
    OutputId   imu_output;
    long       imu_stale_after_ms = 250;

    ResourceId brain_imu;   // empty = no Brain VEX IMU mailbox

    long   bias_samples    = 20;
    long   window_ms       = 2000;
    long   max_gap_ms      = 250;
    double still_travel_m  = 0.001;
    double still_rate_dps  = 1.0;
    double max_rate_dps    = 5.0;
    long   evidence_gap_ms = 100;
    double attempt_s       = 60.0;

    long interval_tolerance_ms  = 20;
    long max_pending_ms         = 500;
    long sensor_loss_ms         = 250;
    bool unplace_on_sensor_loss = true;   // false: warn only, the pose is kept

    bool   fusion      = false;
    long   max_wait_ms = 100;
    double translation_floor_m = 0.0, translation_per_m = 0.0;
    double rotation_floor_rad = 0.0, rotation_per_rad = 0.0, rotation_per_m = 0.0;
    double angle_random_walk_rad_per_sqrt_s = 0.0, bias_rad_per_s = 0.0;

    double history_retention_s = 5.0;
    long   history_capacity    = 1024;
    long   history_gap_ms      = 100;
    long   history_attitude_ms = 100;
};

// node is <BrainProfile>. Resources and outputs are checked against the
// built store and catalog.
bool parseBrainProfileConfig(const ConfigNode& node, const ResourceStore& resources,
                             const ResourceCatalog& outputs, BrainProfileConfig& out,
                             std::string& err);

// This Pi's capability checks, after validateRobotProfile. False: reason is
// a gatr2::ProfileReason, detail the wheel or camera index.
bool checkProfileCapabilities(const BrainProfileConfig& config,
                              const gatr2::RobotProfileDoc& profile, uint8_t& reason,
                              uint8_t& detail);

// The waiting Localization: noop estimator, the configured History.
void writeWaitingLocalization(const BrainProfileConfig& config, tinyxml2::XMLDocument& doc);

// Display names of a gatr2::LocalizationTopology and a gatr2::ProfileReason.
const char* profileTopologyName(uint8_t topology);
const char* profileReasonName(uint8_t reason);

// <Profile><Sensors/><Localization/></Profile> for a checked profile, and the
// binding ids it generated (encoders, imu, bias_function, summary; the
// caller fills id, profile, generation and bench_imu).
void writeProfileSubtrees(const BrainProfileConfig& config, const gatr2::RobotProfileDoc& profile,
                          tinyxml2::XMLDocument& doc, ProfileBinding& binding);

} // namespace navigatr
