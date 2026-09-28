// field_documents.cpp

#include "impl/publishing/field_documents.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <utility>

#include "math/angles.h"
#include "tinyxml2/tinyxml2.h"

namespace navigatr
{

namespace
{

// slack for decimal meters that are whole mm, e.g. 0.095 * 1000
constexpr double kMmSlack = 1e-6;

bool roundMm(double m, int32_t& out) {
    const double mm = std::round(m * 1000.0);
    if (!std::isfinite(mm) || mm < -2147483648.0 || mm > 2147483647.0) {
        return false;
    }
    out = static_cast<int32_t>(mm);
    return true;
}

bool boxOffsetMm(double m, int16_t& out) {
    const double mm = std::round(m * 1000.0);
    if (!std::isfinite(mm) || mm < -32768.0 || mm > 32767.0) {
        return false;
    }
    out = static_cast<int16_t>(mm);
    return true;
}

bool boxSizeMm(double m, uint16_t& out) {
    const double mm = std::ceil(m * 1000.0 - kMmSlack);
    if (!std::isfinite(mm) || mm < 1.0 || mm > 65535.0) {
        return false;
    }
    out = static_cast<uint16_t>(mm);
    return true;
}

bool boundaryMm(double m, bool lower, int32_t& out) {
    const double mm = lower ? std::ceil(m * 1000.0 - kMmSlack) : std::floor(m * 1000.0 + kMmSlack);
    if (!std::isfinite(mm) || mm < -2147483648.0 || mm > 2147483647.0) {
        return false;
    }
    out = static_cast<int32_t>(mm);
    return true;
}

bool fillPose(const Pose2D& pose, gatr2::FieldObjectRecord& r) {
    r.heading_cdeg = radToCdeg(pose.heading_rad);
    return std::isfinite(pose.heading_rad) && roundMm(pose.x_m, r.x_mm) &&
           roundMm(pose.y_m, r.y_mm);
}

bool fillBox(const CollisionBoxDecl& box, gatr2::FieldObjectRecord& r) {
    if (!box.declared) {
        return true;
    }
    if (!std::isfinite(box.center.heading_rad)) {
        return false;
    }
    r.flags |= gatr2::kObjectObstacle;
    r.box_heading_cdeg = static_cast<int16_t>(radToCdeg(box.center.heading_rad));
    return boxOffsetMm(box.center.x_m, r.box_x_mm) && boxOffsetMm(box.center.y_m, r.box_y_mm) &&
           boxSizeMm(box.size_x_m, r.box_length_mm) && boxSizeMm(box.size_y_m, r.box_width_mm);
}

bool sameContent(const std::vector<gatr2::FieldEstimateRecord>& a,
                 const std::vector<gatr2::FieldEstimateRecord>& b) {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        const gatr2::FieldEstimateRecord& x = a[i];
        const gatr2::FieldEstimateRecord& y = b[i];
        if (x.object_id != y.object_id || x.source != y.source || x.flags != y.flags ||
            x.x_mm != y.x_mm || x.y_mm != y.y_mm || x.heading_cdeg != y.heading_cdeg) {
            return false;
        }
    }
    return true;
}

// neutral_goal_0_center -> NeutralGoal0Center; empty when it cannot be an
// identifier
std::string camelCase(const std::string& id) {
    std::string out;
    bool        upper = true;
    for (const char c : id) {
        const unsigned char u = static_cast<unsigned char>(c);
        if (!std::isalnum(u)) {
            upper = true;
            continue;
        }
        out += upper ? static_cast<char>(std::toupper(u)) : c;
        upper = false;
    }
    if (out.empty() || !std::isalpha(static_cast<unsigned char>(out[0]))) {
        return "";
    }
    return out;
}

} // namespace

bool loadFieldMapFile(const std::string& path, FieldMap& out, std::string& err) {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
        err = path + ": cannot load xml: " + doc.ErrorStr();
        return false;
    }
    const tinyxml2::XMLElement* root = doc.RootElement();
    if (root == nullptr || std::string(root->Name()) != "Resource" ||
        root->Attribute("type") == nullptr || std::string(root->Attribute("type")) != "field_map") {
        err = path + ": root must be <Resource type=\"field_map\">";
        return false;
    }
    return parseFieldMap(ConfigNode{root}, out, err);
}

bool buildFieldMapDocument(const FieldMap& map, FieldMapDocument& out, std::string& err) {
    if (map.revision == 0) {
        err = "a published field map needs revision=\"N\" on its root";
        return false;
    }
    if (!map.boundary.declared) {
        err = "a published field map needs a Boundary";
        return false;
    }
    const std::size_t count = map.landmarks.size() + map.obstacles.size();
    if (count > gatr2::kFieldMaxObjects) {
        err = "field map has " + std::to_string(count) + " objects; a map document holds " +
              std::to_string(gatr2::kFieldMaxObjects);
        return false;
    }

    std::vector<FieldMapDocument::Object> objects;
    for (const LandmarkDecl& l : map.landmarks) {
        if (l.wire_id == 0) {
            err = "Landmark " + l.id.value + " needs a wire_id to be published";
            return false;
        }
        FieldMapDocument::Object o;
        o.name             = l.id.value;
        o.landmark         = true;
        o.record.object_id = l.wire_id;
        o.record.kind      = gatr2::kObjectLandmark;
        o.record.flags     = gatr2::kObjectEstimated | gatr2::kObjectReference;
        if (!fillPose(l.nominal, o.record) || !fillBox(l.box, o.record)) {
            err = "Landmark " + l.id.value + " pose or CollisionBox is outside the wire range";
            return false;
        }
        objects.push_back(std::move(o));
    }
    for (const ObstacleDecl& ob : map.obstacles) {
        FieldMapDocument::Object o;
        o.name             = ob.id;
        o.record.object_id = ob.wire_id;
        o.record.kind      = gatr2::kObjectFixed;
        if (!fillPose(ob.pose, o.record) || !fillBox(ob.box, o.record)) {
            err = "Obstacle " + ob.id + " pose or CollisionBox is outside the wire range";
            return false;
        }
        objects.push_back(std::move(o));
    }
    std::sort(objects.begin(), objects.end(),
              [](const FieldMapDocument::Object& a, const FieldMapDocument::Object& b) {
                  return a.record.object_id < b.record.object_id;
              });

    gatr2::FieldMapHeader header;
    header.revision     = map.revision;
    header.object_count = static_cast<uint16_t>(objects.size());
    const FieldBoundaryDecl& b = map.boundary;
    if (!boundaryMm(b.min_x_m, true, header.min_x_mm) ||
        !boundaryMm(b.min_y_m, true, header.min_y_mm) ||
        !boundaryMm(b.max_x_m, false, header.max_x_mm) ||
        !boundaryMm(b.max_y_m, false, header.max_y_mm) || header.min_x_mm >= header.max_x_mm ||
        header.min_y_mm >= header.max_y_mm) {
        err = "Boundary is outside the wire range or narrower than 1 mm";
        return false;
    }

    std::vector<uint8_t> bytes(gatr2::fieldMapLen(header.object_count));
    const uint16_t       len = static_cast<uint16_t>(bytes.size());
    bool                 ok  = gatr2::encodeFieldMapHeader(header, bytes.data(), len);
    for (uint16_t i = 0; ok && i < header.object_count; ++i) {
        ok = gatr2::encodeFieldObjectRecord(objects[i].record, i, bytes.data(), len);
    }
    const gatr2::DocError check = ok ? gatr2::validateFieldMap(bytes.data(), len)
                                     : gatr2::DocError::kLength;
    if (check != gatr2::DocError::kNone) {
        err = "field map document failed validation (DocError " +
              std::to_string(static_cast<int>(check)) + ")";
        return false;
    }
    const uint32_t id = gatr2::crc32(bytes.data(), len);
    if (id == 0) {
        err = "field map id is 0, which means no field on the wire; bump the revision";
        return false;
    }
    out.bytes    = std::move(bytes);
    out.map_id   = id;
    out.revision = map.revision;
    out.objects  = std::move(objects);
    return true;
}

std::vector<gatr2::FieldEstimateRecord> fieldEstimateRecords(const FieldMapDocument& map,
                                                            const FieldState& field,
                                                            const RobotState& robot,
                                                            MonotonicTime     now) {
    std::vector<gatr2::FieldEstimateRecord> out;
    out.reserve(map.objects.size());
    for (const FieldMapDocument::Object& o : map.objects) {
        gatr2::FieldEstimateRecord r;
        r.object_id    = o.record.object_id;
        r.source       = gatr2::kEstimateSourceNominal;
        r.flags        = gatr2::kEstimateValid;
        r.x_mm         = o.record.x_mm;
        r.y_mm         = o.record.y_mm;
        r.heading_cdeg = o.record.heading_cdeg;
        if (o.landmark) {
            const auto it = field.objects.find(FieldObjectId{o.name});
            if (it != field.objects.end()) {
                const FieldObjectState& s = it->second;
                // measured in another odometry frame: nominal, like v3
                const bool usable = s.valid && s.source == EstimateSource::kObserved &&
                                    s.odometry_epoch == robot.odometry_epoch &&
                                    s.lastObservedAt.domain == ClockDomain::kHost &&
                                    now.domain == ClockDomain::kHost;
                int32_t x = 0, y = 0;
                if (usable) {
                    const Pose2D p = compose(robot.field_from_odom, s.T_odom_object);
                    if (roundMm(p.x_m, x) && roundMm(p.y_m, y) && std::isfinite(p.heading_rad)) {
                        r.source       = gatr2::kEstimateSourceObserved;
                        r.x_mm         = x;
                        r.y_mm         = y;
                        r.heading_cdeg = radToCdeg(p.heading_rad);
                        r.age_ms       = static_cast<uint16_t>(
                            std::clamp<int64_t>(now - s.lastObservedAt, 0, 65535));
                    }
                }
            }
        }
        out.push_back(r);
    }
    return out;
}

bool fieldReferencesHeader(const FieldMapDocument& map, const std::string& source,
                           std::string& out, std::string& err) {
    char line[160];
    std::string text = "// Generated by navigatr_field_refs from " + source + ". Do not edit.\n"
                       "#pragma once\n"
                       "#include \"investigatr/reference.h\"\n"
                       "struct Field : investigatr::FieldReferences {\n";
    std::snprintf(line, sizeof(line),
                  "    static constexpr investigatr::MapId kMapId    = 0x%08X;\n"
                  "    static constexpr uint16_t           kRevision = %u;\n",
                  static_cast<unsigned>(map.map_id), static_cast<unsigned>(map.revision));
    text += line;
    std::vector<std::string> names = {"Origin", "RobotAtStart", "kMapId", "kRevision"};
    for (const FieldMapDocument::Object& o : map.objects) {
        if ((o.record.flags & gatr2::kObjectReference) == 0) {
            continue;
        }
        const std::string name = camelCase(o.name);
        if (name.empty() || std::find(names.begin(), names.end(), name) != names.end()) {
            err = "object id " + o.name + " does not make a unique C++ name";
            return false;
        }
        names.push_back(name);
        text += "    static constexpr investigatr::Reference " + name +
                " = investigatr::Reference::object(" + std::to_string(o.record.object_id) +
                ", kMapId);\n";
    }
    text += "};\n";
    out = std::move(text);
    return true;
}

// ---- FieldDocuments ----------------------------------------------------------

FieldDocuments::FieldDocuments(FieldMapDocument map, int64_t estimate_period_ms)
    : map_(std::move(map)), period_ms_(estimate_period_ms) {}

uint32_t FieldDocuments::estimateId() const {
    return estimates_.empty() ? 0 : estimates_.back().id;
}

void FieldDocuments::update(const FieldState& field, const RobotState& robot,
                            MonotonicTime now) {
    std::vector<gatr2::FieldEstimateRecord> records =
        fieldEstimateRecords(map_, field, robot, now);
    const uint32_t epoch  = static_cast<uint32_t>(robot.odometry_epoch);
    const uint32_t anchor = static_cast<uint32_t>(robot.anchor_revision);
    if (!estimates_.empty()) {
        const bool same = epoch == content_epoch_ && anchor == content_anchor_ &&
                          sameContent(records, content_);
        if (same || now - taken_at_ < period_ms_) {
            return;
        }
    }

    last_id_ = last_id_ == UINT32_MAX ? 1 : last_id_ + 1;
    gatr2::FieldEstimateHeader header;
    header.object_count    = static_cast<uint16_t>(records.size());
    header.map_id          = map_.map_id;
    header.estimate_id     = last_id_;
    header.odometry_epoch  = epoch;
    header.anchor_revision = anchor;

    Estimate e;
    e.id = last_id_;
    e.bytes.resize(gatr2::fieldEstimateLen(header.object_count));
    const uint16_t len = static_cast<uint16_t>(e.bytes.size());
    gatr2::encodeFieldEstimateHeader(header, e.bytes.data(), len);
    for (uint16_t i = 0; i < header.object_count; ++i) {
        gatr2::encodeFieldEstimateRecord(records[i], i, e.bytes.data(), len);
    }
    e.crc = gatr2::crc32(e.bytes.data(), len);
    estimates_.push_back(std::move(e));
    while (estimates_.size() > kRetainedEstimates) {
        estimates_.pop_front();
    }
    taken_at_       = now;
    content_        = std::move(records);
    content_epoch_  = epoch;
    content_anchor_ = anchor;
}

uint8_t FieldDocuments::read(uint8_t kind, uint32_t doc_id, uint16_t offset, uint8_t max_len,
                             gatr2::BrainReply& reply) const {
    if (max_len == 0) {
        return gatr2::kResultInvalidArgument;
    }
    const std::vector<uint8_t>* bytes = nullptr;
    uint32_t                    id    = 0;
    uint32_t                    crc   = 0;
    if (kind == gatr2::kDocFieldMap) {
        if (doc_id != 0 && doc_id != map_.map_id) {
            return gatr2::kResultStale;
        }
        bytes = &map_.bytes;
        id    = map_.map_id;
        crc   = map_.map_id;
    } else if (kind == gatr2::kDocFieldEstimate) {
        if (estimates_.empty()) {
            return gatr2::kResultUnavailable;
        }
        const Estimate* e = doc_id == 0 ? &estimates_.back() : nullptr;
        for (const Estimate& candidate : estimates_) {
            if (candidate.id == doc_id) {
                e = &candidate;
            }
        }
        if (e == nullptr) {
            return gatr2::kResultStale;
        }
        bytes = &e->bytes;
        id    = e->id;
        crc   = e->crc;
    } else {
        return gatr2::kResultInvalidArgument;
    }
    const std::size_t total = bytes->size();
    if (offset >= total) {
        return gatr2::kResultInvalidArgument;
    }
    const std::size_t n = std::min<std::size_t>(
        {static_cast<std::size_t>(max_len), gatr2::kDocChunkMax, total - offset});
    reply.doc_kind      = kind;
    reply.doc_id        = id;
    reply.doc_total_len = static_cast<uint16_t>(total);
    reply.doc_crc32     = crc;
    reply.doc_offset    = offset;
    reply.data_len      = static_cast<uint8_t>(n);
    std::memcpy(reply.data, bytes->data() + offset, n);
    return gatr2::kResultOk;
}

void FieldDocuments::reset() {
    estimates_.clear();
    content_.clear();
    taken_at_ = MonotonicTime{};
}

} // namespace navigatr
