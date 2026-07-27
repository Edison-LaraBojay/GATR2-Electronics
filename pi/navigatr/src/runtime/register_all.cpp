// register_all.cpp

#include "runtime/register_all.h"

#include "contracts/association.h"
#include "contracts/commands.h"
#include "contracts/localization.h"
#include "contracts/perception.h"
#include "contracts/pose_correction.h"
#include "contracts/preprocessing.h"
#include "contracts/publishing.h"
#include "contracts/sensor.h"
#include "impl/commands/vex_brain_serial.h"
#include "impl/localization/wheel_imu_prediction.h"
#include "impl/noop/noops.h"
#include "impl/preprocessing/configured_collection.h"
#include "impl/preprocessing/imu_normalization.h"
#include "impl/preprocessing/tracking_wheel_odometry.h"
#include "impl/publishing/vex_brain.h"
#include "impl/resources/field_map_resource.h"
#include "impl/world_prediction/landmark_map.h"
#include "impl/resources/pico_telemetry.h"
#include "impl/resources/serial_links.h"
#include "impl/sensors/pico_channels.h"
#include "resources/resource_store.h"

namespace navigatr
{

void register_resources(FunctionRegistry& functions) {
    functions.add<ResourceMakeFunction>(FunctionKey{"resource/linux_serial_link"},
                                        &make_linux_serial_link);
    functions.add<ResourceMakeFunction>(FunctionKey{"resource/memory_link"},
                                        &make_memory_link);
    functions.add<ResourceMakeFunction>(FunctionKey{"resource/file_replay_link"},
                                        &make_file_replay_link);
    functions.add<ResourceMakeFunction>(FunctionKey{"resource/pico_telemetry"},
                                        &make_pico_telemetry);
    functions.add<ResourceMakeFunction>(FunctionKey{"resource/field_map"},
                                        &make_field_map);
}

void register_sensors(FunctionRegistry& functions) {
    functions.add<SensorMakeFunction>(FunctionKey{"sensor/pico_encoder_channel"},
                                      &PicoEncoderChannelSensor::create);
    functions.add<SensorMakeFunction>(FunctionKey{"sensor/pico_imu_channel"},
                                      &PicoImuChannelSensor::create);
}

void register_commands(FunctionRegistry& functions) {
    functions.add<CommandsMakeFunction>(FunctionKey{"commands/noop"}, &makeNoopCommands);
    functions.add<CommandsMakeFunction>(FunctionKey{"commands/vex_brain_serial"},
                                        &VexBrainSerialCommands::create);
}

void register_preprocessing(FunctionRegistry& functions) {
    functions.add<PreprocessingMakeFunction>(FunctionKey{"preprocessing/noop"},
                                             &makeNoopPreprocessing);
    functions.add<PreprocessingMakeFunction>(
        FunctionKey{"preprocessing/configured_collection"}, &ConfiguredCollection::create);
    functions.add<PreprocessorMakeFunction>(
        FunctionKey{"preprocessor/tracking_wheel_odometry"},
        &TrackingWheelOdometry::create);
    functions.add<PreprocessorMakeFunction>(FunctionKey{"preprocessor/imu_normalization"},
                                            &ImuNormalization::create);
}

void register_localization(FunctionRegistry& functions) {
    functions.add<LocalizationMakeFunction>(FunctionKey{"localization/noop"},
                                            &makeNoopLocalization);
    functions.add<LocalizationMakeFunction>(FunctionKey{"localization/wheel_imu_prediction"},
                                            &WheelImuPrediction::create);
}

void register_perception(FunctionRegistry& functions) {
    functions.add<PerceptionMakeFunction>(FunctionKey{"perception/noop"},
                                          &makeNoopPerception);
}

void register_association(FunctionRegistry& functions) {
    functions.add<AssociationMakeFunction>(FunctionKey{"association/noop"},
                                           &makeNoopAssociation);
}

void register_pose_correction(FunctionRegistry& functions) {
    functions.add<PoseCorrectionMakeFunction>(FunctionKey{"pose_correction/noop"},
                                              &makeNoopPoseCorrection);
}

void register_world_prediction(FunctionRegistry& functions) {
    functions.add<WorldPredictionMakeFunction>(FunctionKey{"world_prediction/noop"},
                                               &makeNoopWorldPrediction);
    functions.add<WorldPredictionMakeFunction>(FunctionKey{"world_prediction/landmark_map"},
                                               &LandmarkMapWorldPrediction::create);
}

void register_publishers(FunctionRegistry& functions) {
    functions.add<PublishingMakeFunction>(FunctionKey{"publishing/noop"},
                                          &makeNoopPublishing);
    functions.add<PublishingMakeFunction>(FunctionKey{"publishing/vex_brain"},
                                          &VexBrainPublisher::create);
}

void registerAll(FunctionRegistry& functions) {
    register_resources(functions);
    register_sensors(functions);
    register_commands(functions);
    register_preprocessing(functions);
    register_localization(functions);
    register_perception(functions);
    register_association(functions);
    register_pose_correction(functions);
    register_world_prediction(functions);
    register_publishers(functions);
}

} // namespace navigatr
