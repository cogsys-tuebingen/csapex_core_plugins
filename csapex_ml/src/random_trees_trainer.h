#ifndef RANDOM_TREES_TRAINER_H
#define RANDOM_TREES_TRAINER_H

/// PROJECT
#include "machine_learning_node.h"
#include <csapex_core_plugins/collection_node.h>
#include <csapex_ml/features_message.h>
namespace csapex
{
class CSAPEX_EXPORT_PLUGIN RandomTreesTrainer : public MachineLearningNode
{
public:
    RandomTreesTrainer();

    void setupParameters(Parameterizable& parameters) override;

private:
    void updatePriors();
    void udpatePriorValues();

    bool processCollection(std::vector<connection_types::FeaturesMessage>& collection) override;

private:
    int categories_;
    std::vector<csapex::param::ParameterPtr> priors_params_;
    std::vector<float> priors_;
    //    std::string        path_;
};

}  // namespace csapex

#endif  // RANDOM_TREES_TRAINER_H
