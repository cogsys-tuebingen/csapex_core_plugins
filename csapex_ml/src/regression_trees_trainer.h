#ifndef REGRESSION_TREES_TRAINER_H
#define REGRESSION_TREES_TRAINER_H

/// PROJECT
#include <csapex_core_plugins/collection_node.h>
#include <csapex_ml/features_message.h>
#include "machine_learning_node.h"

namespace csapex
{


class CSAPEX_EXPORT_PLUGIN RegressionTreesTrainer: public MachineLearningNode//public CollectionNode<connection_types::FeaturesMessage>
{
public:
    RegressionTreesTrainer();

    void setupParameters(csapex::Parameterizable& params) override;

private:
    bool processCollection(std::vector<connection_types::FeaturesMessage> &collection) override;

private:



};
}
#endif // REGRESSION_TREES_TRAINER_H
