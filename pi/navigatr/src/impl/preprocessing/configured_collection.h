// configured_collection.h
// Composite preprocessing: runs the PreprocessingMap of configured
// executables in declaration order. The outer pipeline sees one
// Preprocessing; only this implementation knows it owns a collection.
// Duplicate preprocessor ids and duplicate artifact output ids are
// configuration errors, and an empty collection must be written as the
// preprocessing noop instead.
//
//   <Preprocessing type="configured_collection">
//       <Preprocessor id="tracking_motion" type="tracking_wheel_odometry">
//           ...
//       </Preprocessor>
//   </Preprocessing>

#pragma once
#include <memory>
#include <vector>

#include "contracts/preprocessing.h"
#include "impl/preprocessing/preprocessing_map.h"

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
    PreprocessingMap executables_;
};

} // namespace navigatr
