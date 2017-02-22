#pragma once

#include <csapex_core_plugins/collection_node.h>
#include <csapex_ml/features_message.h>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN  MLPCvTrainer : public CollectionNode<connection_types::FeaturesMessage>
{
public:
    MLPCvTrainer();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;

private:
    bool processCollection(std::vector<connection_types::FeaturesMessage>& collection) override;
    void updateLayers();

private:
    int layers_;
    std::vector<csapex::param::ParameterPtr> layer_params_;
};
}
