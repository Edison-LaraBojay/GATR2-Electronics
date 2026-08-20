// noops.cpp

#include "impl/noop/noops.h"

namespace navigatr
{

namespace
{

class NoopCommands : public Commands
{
public:
    CommandsOutput run(const CommandsInput& in) override {
        return CommandsOutput{in.previous, FunctionStatus::kOk};
    }
};

class NoopPreprocessing : public Preprocessing
{
public:
    PreprocessingOutput run(const PreprocessingInput&) override {
        return PreprocessingOutput{};
    }
};

class NoopLocalization : public Localization
{
public:
    LocalizationOutput run(const LocalizationInput& in) override {
        return LocalizationOutput{in.previous, FunctionStatus::kOk};
    }
};

class NoopPerception : public Perception
{
public:
    PerceptionOutput run(const PerceptionInput&) override { return PerceptionOutput{}; }
};

class NoopAssociation : public Association
{
public:
    AssociationOutput run(const AssociationInput&) override { return AssociationOutput{}; }
};

class NoopWorldEstimation : public WorldEstimation
{
public:
    WorldEstimationOutput run(const WorldEstimationInput& in) override {
        WorldEstimationOutput out;
        out.world = in.previousWorld;
        return out;
    }
};

class NoopTargetResolution : public TargetResolution
{
public:
    TargetResolutionOutput run(const TargetResolutionInput& in) override {
        return TargetResolutionOutput{in.previous, FunctionStatus::kOk};
    }
};

class NoopPublishing : public Publishing
{
public:
    PublishingOutput run(const PublishingInput&) override { return PublishingOutput{}; }
};

} // namespace

std::unique_ptr<Commands> makeNoopCommands(const ConfigNode&, SlotInitializationContext&,
                                           std::string&) {
    return std::make_unique<NoopCommands>();
}

std::unique_ptr<Preprocessing> makeNoopPreprocessing(const ConfigNode&,
                                                     PreprocessorInitializationContext&,
                                                     std::string&) {
    return std::make_unique<NoopPreprocessing>();
}

std::unique_ptr<Localization> makeNoopLocalization(const ConfigNode&,
                                                   SlotInitializationContext&,
                                                   std::string&) {
    return std::make_unique<NoopLocalization>();
}

std::unique_ptr<Perception> makeNoopPerception(const ConfigNode&,
                                               SlotInitializationContext&, std::string&) {
    return std::make_unique<NoopPerception>();
}

std::unique_ptr<Association> makeNoopAssociation(const ConfigNode&,
                                                 SlotInitializationContext&, std::string&) {
    return std::make_unique<NoopAssociation>();
}

std::unique_ptr<WorldEstimation> makeNoopWorldEstimation(const ConfigNode&,
                                                         SlotInitializationContext&,
                                                         std::string&) {
    return std::make_unique<NoopWorldEstimation>();
}

std::unique_ptr<TargetResolution> makeNoopTargetResolution(const ConfigNode&,
                                                           SlotInitializationContext&,
                                                           std::string&) {
    return std::make_unique<NoopTargetResolution>();
}

std::unique_ptr<Publishing> makeNoopPublishing(const ConfigNode&,
                                               SlotInitializationContext&, std::string&) {
    return std::make_unique<NoopPublishing>();
}

} // namespace navigatr
