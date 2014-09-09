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
    CollectionNode<FeaturesMessage>::setupParameters();

    addParameter(param::ParameterFactory::declareFileOutputPath("file", "dtree.yaml"));
}

void DecisionTreeTrainer::processCollection(std::vector<connection_types::FeaturesMessage>& collection)
{
    if(collection.empty()) {
        aerr << "there are no features to train on" << std::endl;
        return;
    }

    FeaturesMessage& first_feature = collection[0];
    std::size_t feature_length = first_feature.value.size();

    cv::Mat train_data(collection.size(), feature_length, CV_32FC1);
    cv::Mat responses(collection.size(), 1, CV_32SC1);
    int tflag = CV_ROW_SAMPLE;

    std::size_t n = collection.size();
    for(std::size_t i = 0; i < n; ++i) {
        FeaturesMessage& feature = collection[0];
        for(std::size_t j = 0; j < feature_length; ++j) {
            train_data.at<float>(i,j) = feature.value[j];
            responses.at<int>(i,0) = feature.classification;
        }
    }

    float p_weight = 1.0;
    float priors[] = { 1, p_weight };
    CvDTreeParams params( 8, // max depth
                          10, // min sample count
                          0, // regression accuracy: N/A here
                          true, // compute surrogate split, as we have missing data
                          15, // max number of categories (use sub-optimal algorithm for larger numbers)
                          10, // the number of cross-validation folds
                          true, // use 1SE rule => smaller tree
                          true, // throw away the pruned tree branches
                          priors);

    cv::Mat var_type( train_data.cols + 1, 1, CV_8U, CV_VAR_NUMERICAL);
    cv::Mat missing;

    cv::DecisionTree dtree;
    ainfo << "starting training with " << n << " features" << std::endl;
    /*bool result = */
    dtree.train(train_data, tflag, responses, cv::Mat(), cv::Mat(), var_type, missing, params);

    ainfo << "training finished, writing tree" << std::endl;
    dtree.save(readParameter<std::string>("file").c_str());
    ainfo << "done writing tree." << std::endl;

//    if(result) {
//    } else {
//        throw std::runtime_error("training failed for an unknown reason");
//    }
}

