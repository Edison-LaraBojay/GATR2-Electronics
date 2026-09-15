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

class NoopStateEstimator : public StateEstimator
{
public:
    StateEstimatorOutput run(const StateEstimatorInput& in) override {
        StateEstimatorOutput out;
        out.robot  = in.previous;
        out.status = FunctionStatus::kOk;
        for (const auto& observation : in.observations) {
            out.rejected.push_back(observation.first);
        }
        return out;
    }
    const std::string& type() const override { return type_; }

private:
    std::string type_ = "noop";
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

class NoopFieldEstimation : public FieldEstimation
{
public:
    FieldEstimationOutput run(const FieldEstimationInput& in) override {
        FieldEstimationOutput out;
        out.field = in.previousField;
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

std::unique_ptr<StateEstimator> makeNoopStateEstimator(const ConfigNode&,
                                                       StateEstimatorInitializationContext&,
                                                       std::string&) {
    return std::make_unique<NoopStateEstimator>();
}

std::unique_ptr<Perception> makeNoopPerception(const ConfigNode&,
                                               SlotInitializationContext&, std::string&) {
    return std::make_unique<NoopPerception>();
}

std::unique_ptr<Association> makeNoopAssociation(const ConfigNode&,
                                                 SlotInitializationContext&, std::string&) {
    return std::make_unique<NoopAssociation>();
}

std::unique_ptr<FieldEstimation> makeNoopFieldEstimation(const ConfigNode&,
                                                         SlotInitializationContext&,
                                                         std::string&) {
    return std::make_unique<NoopFieldEstimation>();
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
