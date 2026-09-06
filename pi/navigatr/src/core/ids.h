// ids.h
// Strongly typed identifiers. A ResourceId is not accepted where a SensorId
// is required, and none of them carry meaning in their spelling: ids are
// opaque identities matched by reference, never parsed for behavior. Values
// come from configuration; there is no global enum of the data in the
// program.
//
// Ids are case sensitive, must be nonempty, and are unique within their own
// typed namespace. They stay stable under XML reordering.

#pragma once
#include <functional>
#include <string>

namespace navigatr
{

template <typename Tag>
struct TypedId {
    std::string value;

    TypedId() = default;
    explicit TypedId(std::string v) : value(std::move(v)) {}

    bool empty() const { return value.empty(); }

    bool operator==(const TypedId& o) const { return value == o.value; }
    bool operator!=(const TypedId& o) const { return value != o.value; }
    bool operator<(const TypedId& o) const { return value < o.value; }

    struct Hash {
        std::size_t operator()(const TypedId& id) const {
            return std::hash<std::string>{}(id.value);
        }
    };
};

using ResourceId     = TypedId<struct ResourceIdTag>;
using SensorId       = TypedId<struct SensorIdTag>;
using PreprocessorId = TypedId<struct PreprocessorIdTag>;
using ArtifactId     = TypedId<struct ArtifactIdTag>;
using ObservationId  = TypedId<struct ObservationIdTag>;
using AssociationId  = TypedId<struct AssociationIdTag>;
using FieldObjectId  = TypedId<struct FieldObjectIdTag>;

// Registered factory key, a plain opaque name like pico_encoder_channel.
using FunctionKey = TypedId<struct FunctionKeyTag>;

// Coordinate frame name, e.g. field, robot, camera. Not a wire packet.
using FrameId = TypedId<struct FrameIdTag>;

} // namespace navigatr
