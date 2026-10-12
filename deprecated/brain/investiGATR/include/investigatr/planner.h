// planner.h
// Path planning, direct or obstacle avoiding, for any MotionModel. A planner
// holds no robot state: each plan depends only on its request.

#pragma once
#include <cstddef>

#include "investigatr/field.h"
#include "investigatr/motion_model.h"
#include "investigatr/path.h"

namespace investigatr
{

// Start and goal refusals: the footprint there overlaps a bound or a box, or
// it is within the enclosing radius and no straight exit gets clear.
enum class PlanStatus : uint8_t {
    kOk,
    kInvalidRequest,   // non-finite pose or invalid model
    kUnsupportedModel, // e.g. non-holonomic without turning in place
    kNoField,          // avoiding without a complete field
    kStartOutOfBounds,
    kStartBlocked,
    kGoalOutOfBounds,
    kGoalBlocked,
    kNoPath, // no route, or the graph would exceed max_vertices
};

const char* toString(PlanStatus status);

struct PlanRequest {
    PlanMode     mode = PlanMode::kDirect;
    Pose         start; // field frame
    Pose         goal;
    MotionModel  model;
    bool         allow_reverse = true;    // non-holonomic, with model.reverse
    const Field* field         = nullptr; // required for kAvoiding, unused by kDirect
};

struct PlanResult {
    PlanStatus status = PlanStatus::kInvalidRequest;
    Path       path;
    ObjectId   blocking = 0; // object blocking the start or goal, 0 = boundary or none
};

class PathPlanner {
public:
    virtual ~PathPlanner() = default;

    virtual PlanResult plan(const PlanRequest& request) const = 0;

    // True when every segment of path stays clear of field for model: the
    // check that decides whether a correction forces a replan. Exact
    // segments are checked by their swept footprint with the clearance;
    // within 2R of the path's start and goal poses (R the enclosing radius)
    // the first and last may come as near as those poses already are, never
    // overlapping at the end.
    virtual bool clear(const Path& path, const Field& field, const MotionModel& model) const = 0;
};

struct GeometricPlannerConfig {
    Meters      vertex_margin = 0.005; // vertices pushed out beyond the grown boxes
    std::size_t max_vertices  = 512;   // graph bound, kNoPath beyond it; work grows as its square
    Meters      min_segment   = 0.005; // shorter translations are dropped
    Radians     min_turn      = 0.01;  // smaller turns dropped (not onto exact moves), <= 0.1
};

// Visibility graph over obstacle boxes grown by the enclosing radius of the
// model, with straight exact exits for a start or goal nearer than that; the
// search starts from every exit and ends at every approach. See
// docs/investigatr.md for the method and its limits.
class GeometricPlanner : public PathPlanner {
public:
    explicit GeometricPlanner(const GeometricPlannerConfig& config = {});

    PlanResult plan(const PlanRequest& request) const override;
    bool       clear(const Path& path, const Field& field, const MotionModel& model) const override;

    const GeometricPlannerConfig& config() const { return config_; }

private:
    GeometricPlannerConfig config_;
};

// Exact test of the footprint rectangle, grown by margin, at pose against
// every obstacle box and the bounds. For monitoring and tests.
struct Clearance {
    bool     clear    = false;
    Meters   distance = 0; // to the nearest obstacle or bound, negative when overlapping
    ObjectId nearest  = 0; // 0 = a bound
};

Clearance footprintClearance(const Pose& robot, const Footprint& footprint, Meters margin,
                             const Field& field);

} // namespace investigatr
