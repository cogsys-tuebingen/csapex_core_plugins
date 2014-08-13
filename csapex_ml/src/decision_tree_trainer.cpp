/// HEADER
#include "decision_tree_trainer.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>

/// SYSTEM
#include <opencv2/ml/ml.hpp>

CSAPEX_REGISTER_CLASS(csapex::DecisionTreeTrainer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


DecisionTreeTrainer::DecisionTreeTrainer()
{
}

void DecisionTreeTrainer::setupParameters()
{
    addParameter(param::ParameterFactory::declareTrigger("train"), boost::bind(&DecisionTreeTrainer::train, this));
}

void DecisionTreeTrainer::setup()
{
    in_  = modifier_->addInput<GenericVectorMessage, FeaturesMessage>("features");
}

void DecisionTreeTrainer::process()
{
    boost::shared_ptr<std::vector<FeaturesMessage> const> features = in_->getMessage<GenericVectorMessage, FeaturesMessage>();
    features_.insert(features_.end(), features->begin(), features->end());
}

void DecisionTreeTrainer::train()
{
    if(features_.empty()) {
        aerr << "there are no features to train on" << std::endl;
        return;
    }

    FeaturesMessage& first_feature = features_[0];
    std::size_t feature_length = first_feature.value.size();

    cv::Mat train_data(features_.size(), feature_length, CV_32FC1);
    cv::Mat responses(features_.size(), 1, CV_32SC1);
    int tflag = CV_ROW_SAMPLE;

    for(std::size_t i = 0, n = features_.size(); i < n; ++i) {
        FeaturesMessage& feature = features_[0];
        for(std::size_t j = 0; j < feature_length; ++j) {
            train_data.at<float>(i,j) = feature.value[j];
            responses.at<int>(i,0) = feature.classification;
        }
    }

    cv::DecisionTree dtree;
    dtree.train(train_data, tflag, responses);
}

