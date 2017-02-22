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

    void setup(NodeModifier &node_modifier) override;
    void setupParameters(Parameterizable &parameters) override;

private:
    /// the output path
    std::string path_;
    /// general settings concerning sample usage
    bool        one_vs_all_;
    bool        balance_;
    enum ClassTypes {NEGATIVE = -1, POSITIVE = 1};
    RandomVector rand_vec_;

    /// decision tree specific settings
    int     classes_;
    int     max_depth_;
    int     min_sample_count_;
    double  regression_accuracy_;
    bool    use_surrogates_;
    int     max_categories_;
    int     cv_folds_;
    bool    use_1se_rule_;
    bool    truncate_pruned_tree_;

    std::vector<csapex::param::ParameterPtr> priors_params_;
    std::vector<float> priors_;

    void updatePriors();
    void updatePriorValues();


    bool processCollection(std::vector<connection_types::FeaturesMessage> &collection) override;

};
}

#endif // DECISION_TREE_FORES_TRAINER_H
