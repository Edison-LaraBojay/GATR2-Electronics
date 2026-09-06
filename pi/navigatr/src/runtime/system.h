// system.h
// The coordinator. Owns the resource store, the configured sensors, and the
// pipeline slot executables, and holds sole execution authority: downstream
// algorithms read the standard result maps, never each other's executables,
// and registry lookup grants nothing at runtime.
//
// The semantic pipeline is fixed:
//
//   Sensor Collection -> Command Collection -> Preprocessing
//     -> Localization -> World Estimation -> Target Resolution -> Publishing
//
// Resources are initialized before runtime and are not a pipeline step.
// Each slot holds one selected implementation behind its contract; an
// implementation may be a leaf, an explicit noop, or a composite that
// privately owns a nested pipeline. The coordinator never learns how many
// internal children exist, so a new composite requires no change here.
// Construction is atomic: parse, register (caller), index and build
// resources, build sensors, build slots resolving every reference with
// payload compatibility, then freeze; any failure destroys the candidate and
// nothing partial ever runs. Shutdown is reverse dependency order through
// member destruction: slots, then sensors, then resources.

#pragma once
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "contracts/commands.h"
#include "contracts/localization.h"
#include "contracts/preprocessing.h"
#include "contracts/publishing.h"
#include "contracts/sensor.h"
#include "contracts/target_resolution.h"
#include "contracts/field_estimation.h"
#include "core/diagnostics.h"
#include "core/function_registry.h"
#include "resources/resource_map.h"
#include "runtime/sensor_catalog.h"
#include "runtime/sensor_map.h"
#include "state/command_state.h"
#include "state/robot_state.h"
#include "state/target_state.h"
#include "state/field_state.h"

namespace navigatr
{

struct BuildOptions {
    // Declared resource id to capture file path. The id must exist in the
    // configuration; its declared implementation is replaced with a replay
    // link for this run.
    std::map<std::string, std::string> replay;

    // Bench-only escape hatch: accept calibration_status="provisional".
    // UNCONFIGURED values are always errors.
    bool allow_provisional = false;
};

class System
{
public:
    static std::unique_ptr<System> buildFromFile(const std::string& path,
                                                 const FunctionRegistry& functions,
                                                 std::string&            err,
                                                 const BuildOptions&     options = {});
    static std::unique_ptr<System> buildFromString(const char*             xml,
                                                   const FunctionRegistry& functions,
                                                   std::string&            err,
                                                   const BuildOptions&     options = {});

    // One full pipeline cycle. now is the host monotonic clock.
    void step(MonotonicTime now);

    // Back to power-on: implementations reset, states cleared, counters kept.
    void reset();

    double   loopRateHz() const { return loop_rate_hz_; }
    uint64_t cycle() const { return cycle_; }

    // Identity of the running profile: the Configuration id (or the file
    // name for a plain System document) and a content digest over every
    // contributing file.
    const std::string& configurationId() const { return configuration_id_; }
    uint64_t           configurationDigest() const { return configuration_digest_; }

    const SensorResultsMap& sensorResults() const { return sensor_results_; }
    const RobotState&       robot() const { return robot_; }
    const FieldState&       field() const { return field_; }
    const CommandState&     command() const { return command_; }
    const TargetState&      target() const { return target_; }
    Diagnostics&            diagnostics() { return diagnostics_; }

    // Non-fatal build notes, e.g. a serial device that failed to open.
    const std::vector<std::string>& warnings() const { return warnings_; }

    // Read access for tests and tooling.
    const ResourceMap& resources() const { return resources_; }
    const SensorCatalog& sensorCatalog() const { return catalog_; }

private:
    System() = default;

    bool build(const char* xml, const FunctionRegistry& functions,
               const BuildOptions& options, std::string& err);

    // Drops entries whose id was never declared or whose payload
    // contradicts the declaration, noting a fault against the producer.
    template <typename Map, typename Decls>
    void enforceDeclared(Map& map, const Decls& decls, const std::string& label) {
        for (auto it = map.begin(); it != map.end();) {
            const auto* decl = [&]() -> const typename Decls::value_type* {
                for (const auto& d : decls) {
                    if (d.id == it->first) {
                        return &d;
                    }
                }
                return nullptr;
            }();
            if (decl == nullptr || !decl->payload.matches(it->second.payload.cppType())) {
                diagnostics_.note(label + "/undeclared_output:" + it->first.value,
                                  FunctionStatus::kFault);
                it = map.erase(it);
            } else {
                ++it;
            }
        }
    }

    double loop_rate_hz_ = 100.0;

    std::string configuration_id_ = "inline";
    uint64_t    configuration_digest_ = 0;

    // destruction order: declared first, destroyed last
    ResourceMap            resources_;
    std::vector<std::string> warnings_;

    SensorMap     sensors_;   // id-keyed executables, deterministic order
    SensorCatalog catalog_;

    // Declared outputs, kept for runtime enforcement: a producer cannot
    // place an undeclared id or a payload contradicting its declaration
    // into a standard map.
    std::vector<ArtifactOutputDecl>    artifact_decls_;
    std::vector<ObservationOutputDecl> observation_decls_;
    std::vector<AssociationOutputDecl> association_decls_;

    std::unique_ptr<Commands>         commands_;
    std::unique_ptr<Preprocessing>    preprocessing_;
    std::unique_ptr<Localization>     localization_;
    std::unique_ptr<FieldEstimation>  field_estimation_;
    std::unique_ptr<TargetResolution> target_resolution_;
    std::unique_ptr<Publishing>       publishing_;
    std::string                       slot_labels_[6];

    uint64_t         cycle_ = 0;
    SensorResultsMap sensor_results_;
    RobotState       robot_;
    FieldState       field_;
    CommandState     command_;
    TargetState      target_;
    Diagnostics      diagnostics_;
};

} // namespace navigatr
