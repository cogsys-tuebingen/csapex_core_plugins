/// HEADER
#include "decision_tree_trainer.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/signal/trigger.h>

/// SYSTEM
#include <opencv2/ml/ml.hpp>

CSAPEX_REGISTER_CLASS(csapex::DecisionTreeTrainer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


DecisionTreeTrainer::DecisionTreeTrainer()
{
}

void DecisionTreeTrainer::setup()
{
    CollectionNode<connection_types::FeaturesMessage>::setup();
}

void DecisionTreeTrainer::setupParameters()
{
    CollectionNode<FeaturesMessage>::setupParameters();

    addParameter(param::ParameterFactory::declareFileOutputPath
                 ("file", "dtree.yaml"));

    addParameter(param::ParameterFactory::declareRange<int>
                 ("max depth",
                  param::ParameterDescription(std::string("The maximum possible depth of the tree. \n") +
                                              "That is the training algorithms attempts to split a node while its depth is less than max_depth. \n" +
                                              "The actual depth may be smaller if the other termination criteria are met \n" +
                                              "(see the outline of the training procedure in the beginning of the section), and/or if the tree is pruned."),
                  1, 64, 8, 1));;
    addParameter(param::ParameterFactory::declareRange<int>
                 ("min sample count",
                  param::ParameterDescription("If the number of samples in a node is less than this parameter then the node will not be split."),
                  0, 64, 10, 1));
    addParameter(param::ParameterFactory::declareRange<double>
                 ("regression accuracy",
                  param::ParameterDescription(std::string("Termination criteria for regression trees. \n") +
                                              "If all absolute differences between an estimated value in a node and values of train samples in this node \n" +
                                              "are less than this parameter then the node will not be split."),
                  0.0, 255.0, 0.0, 0.01));;
    addParameter(param::ParameterFactory::declareBool
                 ("use surrogates",
                  param::ParameterDescription(std::string("If true then surrogate splits will be built. \n") +
                                              "These splits allow to work with missing data and compute variable importance correctly."),
                  true));;
    addParameter(param::ParameterFactory::declareRange<int>
                 ("max categories",
                  param::ParameterDescription(std::string("Cluster possible values of a categorical variable into K < max_categories clusters to find a suboptimal split. \n") +
                                              "If a discrete variable, on which the training procedure tries to make a split, \n" +
                                              "takes more than max_categories values, the precise best subset estimation may take a very long time \n" +
                                              "because the algorithm is exponential. \n" +
                                              "Instead, many decision trees engines (including ML) try to find sub-optimal split in this case by clustering \n" +
                                              "all the samples into max_categories clusters that is some categories are merged together. \n" +
                                              "The clustering is applied only in n>2-class classification problems for categorical variables \n" +
                                              "with N > max_categories possible values. \n" +
                                              "In case of regression and 2-class classification the optimal split can be found efficiently \n" +
                                              "without employing clustering, thus the parameter is not used in these cases."),
                  0, 100, 15, 1));;
    addParameter(param::ParameterFactory::declareRange<int>
                 ("cv folds",
                  param::ParameterDescription("If cv_folds > 1 then prune a tree with K-fold cross-validation where K is equal to cv_folds."),
                  0, 100, 10, 1));;
    addParameter(param::ParameterFactory::declareBool
                 ("use 1se rule",
                  param::ParameterDescription(std::string("If true then a pruning will be harsher.\n") +
                                              "This will make a tree more compact and more resistant to the training data \n" +
                                              "noise but a bit less accurate."),
                  true));;
    addParameter(param::ParameterFactory::declareBool
                 ("truncate pruned tree",
                  param::ParameterDescription(std::string("If true then pruned branches are physically removed from the tree. \n") +
                                              "Otherwise they are retained and it is possible to get results from the \n" +
                                              "original unpruned (or pruned less aggressively) tree by decreasing CvDTree::pruned_tree_idx parameter."),
                  true));;
#if 0
    addParameter(param::ParameterFactory::declareRange<int>
                 ("priors",
                  param::ParameterDescription("The array of a priori class probabilities, sorted by the class label value. \n" +
                                              "The parameter can be used to tune the decision tree preferences toward a certain class. \n" +
                                              "For example, if you want to detect some rare anomaly occurrence, \n" +
                                              "the training base will likely contain much more normal cases than anomalies, \n" +
                                              "so a very good classification performance will be achieved just by considering every case as normal. \n" +
                                              "To avoid this, the priors can be specified, where the anomaly probability is artificially increased \n" +
                                              "(up to 0.5 or even greater), so the weight of the misclassified anomalies becomes much bigger, \n" +
                                              "and the tree is adjusted properly. \n" +
                                              "You can also think about this parameter as weights of prediction categories which determine \n" +
                                              "relative weights that you give to misclassification. \n" +
                                              "That is, if the weight of the first category is 1 and the weight of the second category is 10, \n" +
                                              "then each mistake in predicting the second category is equivalent to \n" +
                                              "making 10 mistakes in predicting the first category.",
                                              ));;
    )
#endif
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
    cv::Mat missing(collection.size(), feature_length, CV_8UC1, cv::Scalar(0));

    cv::Mat responses(collection.size(), 1, CV_32SC1);
    int tflag = CV_ROW_SAMPLE;

    std::size_t n = collection.size();
    for(std::size_t i = 0; i < n; ++i) {
        FeaturesMessage& feature = collection[i];
        for(std::size_t j = 0; j < feature_length; ++j) {
            const float& val = feature.value[j];

            if(std::abs(val) >= FLT_MAX*0.5f) {
                missing.at<uchar>(i,j) = 1;
            } else {
                train_data.at<float>(i,j) = val;
            }
        }

        responses.at<int>(i,0) = feature.classification;
    }

    float p_weight = 1.0;
    float priors[] = { 1, p_weight };
    // TODO: optimize with eva
    CvDTreeParams params( readParameter<int>("max depth"),
                          readParameter<int>("min sample count"),
                          readParameter<double>("regression accuracy"),
                          readParameter<bool>("use surrogates"),
                          readParameter<int>("max categories"),
                          readParameter<int>("cv folds"),
                          readParameter<bool>("use 1se rule"),
                          readParameter<bool>("truncate pruned tree"),
                          priors);

    cv::Mat var_type( train_data.cols + 1, 1, CV_8U, CV_VAR_NUMERICAL);

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

