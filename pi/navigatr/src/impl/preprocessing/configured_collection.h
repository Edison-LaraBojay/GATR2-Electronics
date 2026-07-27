// configured_collection.h
// Composite preprocessing: runs the configured preprocessors in declaration
// order. The outer pipeline sees one Preprocessing; only this implementation
// knows it owns a collection. Duplicate preprocessor ids and duplicate
// artifact output ids are configuration errors, and an empty collection must
// be written as preprocessing/noop instead.
//
//   <Preprocessing type="preprocessing/configured_collection">
//       <Preprocessor id="tracking_motion"
//                     type="preprocessor/tracking_wheel_odometry">
//           ...
//       </Preprocessor>
//   </Preprocessing>

#pragma once
#include <memory>
#include <vector>

#include "contracts/preprocessing.h"

namespace navigatr
{

class ConfiguredCollection : public Preprocessing
{
public:
    static std::unique_ptr<Preprocessing> create(const ConfigNode& node,
                                                 PreprocessorInitializationContext& context,
                                                 std::string& err);

    PreprocessingOutput run(const PreprocessingInput& in) override;

    std::vector<ArtifactOutputDecl> produces() const override;

    void reset() override;

private:
    std::vector<std::unique_ptr<PreprocessorExecutable>> executables_;
};

} // namespace navigatr
