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

class NoopPoseCorrection : public PoseCorrection
{
public:
    PoseCorrectionOutput run(const PoseCorrectionInput& in) override {
        return PoseCorrectionOutput{in.predicted, FunctionStatus::kOk};
    }
};

class NoopWorldPrediction : public WorldPrediction
{
public:
    WorldPredictionOutput run(const WorldPredictionInput& in) override {
        return WorldPredictionOutput{in.previousWorld, in.previousTarget,
                                     FunctionStatus::kOk};
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

std::unique_ptr<PoseCorrection> makeNoopPoseCorrection(const ConfigNode&,
                                                       SlotInitializationContext&,
                                                       std::string&) {
    return std::make_unique<NoopPoseCorrection>();
}

std::unique_ptr<WorldPrediction> makeNoopWorldPrediction(const ConfigNode&,
                                                         SlotInitializationContext&,
                                                         std::string&) {
    return std::make_unique<NoopWorldPrediction>();
}

std::unique_ptr<Publishing> makeNoopPublishing(const ConfigNode&,
                                               SlotInitializationContext&, std::string&) {
    return std::make_unique<NoopPublishing>();
}

} // namespace navigatr
