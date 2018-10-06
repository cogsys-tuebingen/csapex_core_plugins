#ifndef WALDBOOSTTRAINER_H
#define WALDBOOSTTRAINER_H

/// COMPONENT
#include "waldboost/waldboost.hpp"
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex_core_plugins/collection_node.h>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN WaldBoostTrainer : public CollectionNode<connection_types::FeaturesMessage>
{
public:
    WaldBoostTrainer();

    void setup(NodeModifier& modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    std::string path_;
    int weak_count_;

    bool processCollection(std::vector<connection_types::FeaturesMessage>& collection) override;
};
}  // namespace csapex

#endif  // WALDBOOSTTRAINER_H
