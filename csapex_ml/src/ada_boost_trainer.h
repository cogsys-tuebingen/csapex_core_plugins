#ifndef ADABOOSTTRAINER_H
#define ADABOOSTTRAINER_H

/// COMPONENT
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex_core_plugins/collection_node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN AdaBoostTrainer : public CollectionNode<connection_types::FeaturesMessage>
{
public:
    AdaBoostTrainer();

    void setupParameters(Parameterizable& parameters) override;

private:
    std::string path_;

    int boost_type_;
    int weak_count_;
    int split_criteria_;

    int max_depth_;
    bool use_surrogates_;

    double weight_trim_rate_;

    bool processCollection(std::vector<connection_types::FeaturesMessage>& collection) override;
};
}  // namespace csapex

#endif  // ADABOOSTTRAINER_H
