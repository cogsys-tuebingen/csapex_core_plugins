#ifndef DECISION_TREE_TRAINER_H
#define DECISION_TREE_TRAINER_H

/// PROJECT
#include <csapex_core_plugins/collection_node.h>
#include <csapex_ml/features_message.h>

namespace csapex {


class CSAPEX_EXPORT_PLUGIN DecisionTreeTrainer : public CollectionNode<connection_types::FeaturesMessage>
{
public:
    DecisionTreeTrainer();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    void updatePriors();
    void udpatePriorValues();

    void processCollection(std::vector<connection_types::FeaturesMessage> &collection) override;

private:
    int categories_;
    std::vector<csapex::param::ParameterPtr> priors_params_;
    std::vector<float> priors_;

};


}

#endif // DECISION_TREE_TRAINER_H
