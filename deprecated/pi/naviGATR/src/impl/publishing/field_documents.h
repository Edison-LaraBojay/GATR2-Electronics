// field_documents.h
// The brain link field documents (translaGATR/link_documents.h) of one field
// definition: the map, built once, and the estimate snapshots taken in the
// reporting cycle and retained for chunked READ_DOC. Wire units are owned
// here: positions round to the nearest mm and headings to the centidegree;
// box sizes round up and the boundary rounds inward to whole mm, so the
// wire never shrinks an obstacle or grows the drivable region.

#pragma once
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "translaGATR/frames.h"
#include "translaGATR/link_documents.h"
#include "config/field_map.h"
#include "core/time.h"
#include "state/field_state.h"
#include "state/robot_state.h"

namespace navigatr
{

struct FieldMapDocument {
    struct Object {
        std::string              name;   // Landmark or Obstacle id in the definition
        bool                     landmark = false;
        translagatr::FieldObjectRecord record;
    };

    std::vector<uint8_t> bytes;
    uint32_t             map_id   = 0;   // crc32(bytes)
    uint16_t             revision = 0;
    std::vector<Object>  objects;   // map order: ascending wire id
};

// Parses a field definition file whose root is <Resource type="field_map">.
bool loadFieldMapFile(const std::string& path, FieldMap& out, std::string& err);

// Landmarks become kind landmark, flags estimated and reference, plus
// obstacle with a CollisionBox; Obstacles become kind fixed, flag obstacle
// with a CollisionBox. False and err when the definition cannot be
// published: no revision or Boundary, a landmark without wire_id, more
// objects than a document holds, or a value outside its wire range.
bool buildFieldMapDocument(const FieldMap& map, FieldMapDocument& out, std::string& err);

// One record per map object. A landmark is observed when the field state
// holds a valid observed estimate from the robot's odometry epoch: its pose
// is composed with the current anchor and its age is now minus the newest
// observation. Every other record is nominal and valid with age 0, which is
// every record when world estimation is noop.
std::vector<translagatr::FieldEstimateRecord> fieldEstimateRecords(const FieldMapDocument& map,
                                                            const FieldState& field,
                                                            const RobotState& robot,
                                                            MonotonicTime     now);

// Brain header naming every reference object of the map,
//   struct Field : investigatr::FieldReferences { ... };
// with the map id and revision. source names the field file in the first
// line. False and err when an id does not make a unique C++ identifier.
bool fieldReferencesHeader(const FieldMapDocument& map, const std::string& source,
                           std::string& out, std::string& err);

// The map and the newest estimate snapshots of one publisher.
class FieldDocuments
{
public:
    static constexpr std::size_t kRetainedEstimates = 3;

    FieldDocuments(FieldMapDocument map, int64_t estimate_period_ms);

    // Takes estimate_id + 1 when the records (source, validity, wire pose),
    // the odometry epoch or the anchor revision changed and at least
    // estimate_period_ms passed since the last one taken. The first call
    // always takes one.
    void update(const FieldState& field, const RobotState& robot, MonotonicTime now);

    uint32_t                mapId() const { return map_.map_id; }
    uint32_t                estimateId() const;   // newest, 0 = none yet
    const FieldMapDocument& map() const { return map_; }

    // One READ_DOC chunk into the reply's document fields; returns the
    // result. doc_id 0 is the current document. Stale when doc_id is not
    // retained, Unavailable before the first estimate, InvalidArgument for
    // an unknown kind, max_len 0, or offset at or past the end.
    uint8_t read(uint8_t kind, uint32_t doc_id, uint16_t offset, uint8_t max_len,
                 translagatr::BrainReply& reply) const;

    // Drops the estimates. Ids keep counting, so none is reused.
    void reset();

private:
    struct Estimate {
        uint32_t             id  = 0;
        uint32_t             crc = 0;
        std::vector<uint8_t> bytes;
    };

    FieldMapDocument                        map_;
    int64_t                                 period_ms_ = 0;
    std::deque<Estimate>                    estimates_;   // oldest first
    uint32_t                                last_id_ = 0;
    MonotonicTime                           taken_at_;
    std::vector<translagatr::FieldEstimateRecord> content_;   // records of the newest
    uint32_t                                content_epoch_  = 0;
    uint32_t                                content_anchor_ = 0;
};

} // namespace navigatr
