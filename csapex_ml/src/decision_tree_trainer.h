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

    bool processCollection(std::vector<connection_types::FeaturesMessage> &collection) override;

private:
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

    std::string path_;

};


}

#endif // DECISION_TREE_TRAINER_H
