#ifndef ADABOOSTTRAINER_H
#define ADABOOSTTRAINER_H

/// COMPONENT
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex_core_plugins/collection_node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {
class CSAPEX_EXPORT_PLUGIN  AdaBoostTrainer : public CollectionNode<connection_types::FeaturesMessage>
{
public:
    AdaBoostTrainer();

    void setupParameters(Parameterizable &parameters) override;

private:
    std::string     path_;

    int boost_type;
    int weak_count;
    int split_criteria;

    int max_depth;
    bool use_surrogates;


    double          weight_trim_rate_;

    bool processCollection(std::vector<connection_types::FeaturesMessage> &collection) override;
};
}

#endif // ADABOOSTTRAINER_H
