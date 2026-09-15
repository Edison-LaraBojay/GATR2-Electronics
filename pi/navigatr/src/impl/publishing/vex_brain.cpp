// vex_brain.cpp

#include "impl/publishing/vex_brain.h"

#include <algorithm>
#include <cmath>
#include <typeindex>

#include "common/frame_codec.h"
#include "math/angles.h"
#include "payloads/landmark_associations.h"
#include "resources/resource_store.h"
#include "runtime/sensor_catalog.h"

namespace navigatr
{

namespace
{

int32_t toWireMm(double meters) { return static_cast<int32_t>(std::llround(meters * 1000.0)); }

int16_t clampWireMm16(double meters) {
    const double mm = static_cast<double>(std::llround(meters * 1000.0));
    if (mm > 32767.0) {
        return 32767;
    }
    if (mm < -32768.0) {
        return -32768;
    }
    return static_cast<int16_t>(mm);
}

uint8_t quality255(double q) {
    const double r = static_cast<double>(std::llround(q * 255.0));
    if (r > 255.0) {
        return 255;
    }
    if (r < 0.0) {
        return 0;
    }
    return static_cast<uint8_t>(r);
}

} // namespace

std::unique_ptr<Publishing> VexBrainPublisher::create(const ConfigNode& node,
                                                      SlotInitializationContext& context,
                                                      std::string& err) {
    auto publisher = std::make_unique<VexBrainPublisher>();

    const ResourceId link_id{node.child("Serial").attr("resource_id")};
    if (link_id.empty()) {
        err = node.path() + ": needs <Serial resource_id=.../>";
        return nullptr;
    }
    if (context.resources == nullptr || context.sensors == nullptr) {
        err = node.path() + ": no resources available";
        return nullptr;
    }
    std::string inner;
    publisher->link_ = context.resources->require<SerialLink>(link_id, inner);
    if (publisher->link_ == nullptr) {
        err = node.path() + ": " + inner;
        return nullptr;
    }

    const ConfigNode health = node.child("Health");
    if (health.valid()) {
        if (!health.getInt("fresh_ms", 150, publisher->fresh_ms_, err)) {
            return nullptr;
        }
        if (publisher->fresh_ms_ <= 0) {
            err = health.path() + ": fresh_ms must be positive";
            return nullptr;
        }
        bool ok = true;
        health.forEach("Encoder", [&](const ConfigNode& e) {
            if (!ok) {
                return;
            }
            const SensorId id{e.attr("sensor_id")};
            if (context.sensors->payloadOf(id) == nullptr) {
                err = e.path() + ": references sensor " + id.value +
                      " which is not configured";
                ok = false;
                return;
            }
            publisher->encoder_health_.push_back(id);
        });
        if (!ok) {
            return nullptr;
        }
        const ConfigNode gyro = health.child("Gyro");
        if (gyro.valid()) {
            publisher->gyro_health_ = SensorId{gyro.attr("sensor_id")};
            if (context.sensors->payloadOf(publisher->gyro_health_) == nullptr) {
                err = gyro.path() + ": references sensor " +
                      publisher->gyro_health_.value + " which is not configured";
                return nullptr;
            }
        }
        const ConfigNode bias = health.child("BiasCal");
        if (bias.valid()) {
            publisher->bias_cal_function_ = bias.attr("function_id");
            if (publisher->bias_cal_function_.empty()) {
                err = bias.path() + ": BiasCal needs function_id";
                return nullptr;
            }
            if (!context.requireObservationFunction(publisher->bias_cal_function_,
                                                    bias.path(), err)) {
                return nullptr;
            }
        }
    }

    bool ok = true;
    node.forEach("FieldObject", [&](const ConfigNode& w) {
        if (!ok) {
            return;
        }
        WireObject wire;
        wire.object = FieldObjectId{w.attr("object_id")};
        long id     = -1;
        if (wire.object.empty() || !w.getInt("wire_id", -1, id, err)) {
            if (err.empty()) {
                err = w.path() + ": FieldObject needs object_id and wire_id";
            }
            ok = false;
            return;
        }
        if (id < 0 || id > 255) {
            err = w.path() + ": wire_id must be 0..255";
            ok  = false;
            return;
        }
        wire.wire_id = static_cast<uint8_t>(id);
        for (const WireObject& seen : publisher->wire_objects_) {
            if (seen.wire_id == wire.wire_id || seen.object == wire.object) {
                err = w.path() + ": duplicate FieldObject mapping";
                ok  = false;
                return;
            }
        }
        publisher->wire_objects_.push_back(wire);
    });
    if (!ok) {
        return nullptr;
    }

    const ConfigNode landmarks = node.child("Landmarks");
    if (landmarks.valid()) {
        publisher->landmarks_ref_ = AssociationId{landmarks.attr("association_id")};
        const std::type_index expected(typeid(LandmarkAssociationSet));
        if (!context.requireAssociation(publisher->landmarks_ref_, &expected,
                                        landmarks.path(), err)) {
            return nullptr;
        }
    }
    return publisher;
}

void VexBrainPublisher::reset() {
    bias_cal_seen_ = false;
    seq_           = 0;
}

bool VexBrainPublisher::sensorFresh(const SensorMap& results, const SensorId& id,
                                    MonotonicTime now) const {
    const auto it = results.find(id);
    if (it == results.end() || it->second.state != SourceState::kValid ||
        !it->second.latest.has_value()) {
        return false;
    }
    return (now - it->second.latest->receivedAt) <= fresh_ms_;
}

PublishingOutput VexBrainPublisher::run(const PublishingInput& in) {
    PublishingOutput out;
    if (!in.command.stream_on) {
        return out;
    }

    if (!bias_cal_function_.empty()) {
        const ObservationFunctionStatus* f = in.localization.find(bias_cal_function_);
        if (f != nullptr && f->ready) {
            bias_cal_seen_ = true;
        }
    }

    const Pose2D field_pose = in.robot.fieldPose();

    gatr2::PoseFrame p{};
    p.seq          = seq_++;
    p.stamp_ms     = static_cast<uint32_t>(in.robot.measuredAt.ms);
    p.x_mm         = toWireMm(field_pose.x_m);
    p.y_mm         = toWireMm(field_pose.y_m);
    p.heading_cdeg = radToCdeg(field_pose.heading_rad);

    uint16_t status = 0;
    if (in.robot.valid) {
        status |= gatr2::kStatusPoseValid;
    }
    if (in.robot.initialized) {
        status |= gatr2::kStatusLocInit;
    }
    if (!encoder_health_.empty()) {
        bool all_fresh = true;
        for (const SensorId& id : encoder_health_) {
            all_fresh = all_fresh && sensorFresh(in.sensors, id, in.now);
        }
        if (all_fresh) {
            status |= gatr2::kStatusEncHealthy;
        }
    }
    if (!gyro_health_.empty() && sensorFresh(in.sensors, gyro_health_, in.now)) {
        status |= gatr2::kStatusGyroHealthy;
    }
    if (!in.observations.empty()) {
        status |= gatr2::kStatusVisionAlive;
    }
    if (bias_cal_seen_) {
        status |= gatr2::kStatusBiasCal;
    }

    if (in.command.object_requested) {
        status |= gatr2::kStatusObjRequested;
        p.object_id = in.command.object_wire_id;

        // An active latched target for the requested wire id wins: the brain
        // receives the desired robot body pose, field frame. Otherwise fall
        // back to a configured world object estimate.
        bool published = false;
        if (in.target.active && in.target.latched &&
            in.target.status != TargetStatus::kCancelled &&
            in.target.wire_id == in.command.object_wire_id) {
            const Pose2D target_field =
                compose(in.robot.field_from_odom, in.target.T_odom_robot_target);
            status |= gatr2::kStatusObjValid;
            if (in.target.status == TargetStatus::kLockedVision) {
                status |= gatr2::kStatusObjObserved;
            }
            p.obj_x_mm         = toWireMm(target_field.x_m);
            p.obj_y_mm         = toWireMm(target_field.y_m);
            p.obj_heading_cdeg = radToCdeg(target_field.heading_rad);
            published          = true;
        }
        if (!published) {
            for (const WireObject& wire : wire_objects_) {
                if (wire.wire_id != in.command.object_wire_id) {
                    continue;
                }
                const auto it = in.field.objects.find(wire.object);
                if (it != in.field.objects.end() && it->second.valid) {
                    status |= gatr2::kStatusObjValid;
                    if (it->second.observed) {
                        status |= gatr2::kStatusObjObserved;
                    }
                    p.obj_x_mm         = toWireMm(it->second.pose.pose.x_m);
                    p.obj_y_mm         = toWireMm(it->second.pose.pose.y_m);
                    p.obj_heading_cdeg = radToCdeg(it->second.pose.pose.heading_rad);
                }
                break;
            }
        }
    }
    p.status = status;

    if (!landmarks_ref_.empty()) {
        const auto it = in.associations.find(landmarks_ref_);
        if (it != in.associations.end()) {
            const LandmarkAssociationSet* set =
                it->second.payload.get<LandmarkAssociationSet>();
            if (set != nullptr && !set->entries.empty()) {
                auto sorted = set->entries;
                std::sort(sorted.begin(), sorted.end(),
                          [](const LandmarkAssociationEntry& a,
                             const LandmarkAssociationEntry& b) {
                              return a.quality > b.quality;
                          });
                std::size_t n = 0;
                for (const LandmarkAssociationEntry& entry : sorted) {
                    if (n >= gatr2::kMaxLandmarks) {
                        break;
                    }
                    // only objects with a configured wire id can go on the wire
                    const WireObject* wire = nullptr;
                    for (const WireObject& candidate : wire_objects_) {
                        if (candidate.object == entry.landmark) {
                            wire = &candidate;
                            break;
                        }
                    }
                    if (wire == nullptr) {
                        continue;
                    }
                    gatr2::LandmarkObs& L = p.landmarks[n++];
                    L.id                  = wire->wire_id;
                    L.dx_mm               = clampWireMm16(entry.dx_m);
                    L.dy_mm               = clampWireMm16(entry.dy_m);
                    L.bearing_cdeg = static_cast<int16_t>(radToCdeg(entry.bearing_rad));
                    L.quality      = quality255(entry.quality);
                }
                p.n_landmarks = static_cast<uint8_t>(n);
            }
        }
    }

    uint8_t        buf[gatr2::kMaxFrameLen];
    const uint16_t len = gatr2::encodePoseFrame(p, buf, sizeof(buf));
    if (len == 0 || !link_->write(ByteSpan{buf, len}).ok) {
        out.status = FunctionStatus::kFault;
    }
    return out;
}

} // namespace navigatr
