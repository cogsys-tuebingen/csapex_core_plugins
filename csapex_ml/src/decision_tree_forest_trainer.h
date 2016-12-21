#ifndef DECISION_TREE_FORES_TRAINER_H
#define DECISION_TREE_FORES_TRAINER_H

/// COMPONENT
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex_core_plugins/collection_node.h>
#include "random_vector.hpp"

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {
class CSAPEX_EXPORT_PLUGIN DecisionTreeForestTrainer : public CollectionNode<connection_types::FeaturesMessage>
{
public:
    DecisionTreeForestTrainer();

    void setup(NodeModifier &modifier) override;
    void setupParameters(Parameterizable &parameters) override;

private:
    std::string path_;
    bool        one_vs_all_;
    bool        balance_;

    int categories_;
    std::vector<csapex::param::ParameterPtr> priors_params_;
    std::vector<float> priors_;

    enum ClassTypes {NEGATIVE = -1, POSITIVE = 1};

    bool processCollection(std::vector<connection_types::FeaturesMessage> &collection) override;
    RandomVector rand_vec_;

};
}

#endif // DECISION_TREE_FORES_TRAINER_H
