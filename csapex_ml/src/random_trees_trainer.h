#ifndef RANDOM_TREES_TRAINER_H
#define RANDOM_TREES_TRAINER_H

/// PROJECT
#include <csapex_core_plugins/vector_collection_node.h>
#include <csapex_ml/features_message.h>

namespace csapex {


class RandomTreesTrainer : public VectorCollectionNode<connection_types::FeaturesMessage>
{
public:
    RandomTreesTrainer();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters);

private:
    void updatePriors();
    void udpatePriorValues();

    void processCollection(std::vector<connection_types::FeaturesMessage> &collection);

private:
    int categories_;
    std::vector<csapex::param::ParameterPtr> priors_params_;
    std::vector<float> priors_;
};


}

#endif // RANDOM_TREES_TRAINER_H
