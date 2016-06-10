/// HEADER
#include "decision_tree_trainer.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/signal/event.h>

/// SYSTEM
#include <opencv2/ml/ml.hpp>

CSAPEX_REGISTER_CLASS(csapex::DecisionTreeTrainer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


DecisionTreeTrainer::DecisionTreeTrainer()
    : categories_(0)
{
}

void DecisionTreeTrainer::setup(NodeModifier& node_modifier)
{
    CollectionNode<connection_types::FeaturesMessage>::setup(node_modifier);
}

void DecisionTreeTrainer::setupParameters(Parameterizable& parameters)
{
    CollectionNode<FeaturesMessage>::setupParameters(parameters);

    addParameter(csapex::param::ParameterFactory::declareRange<int>
                 ("classes",
                  csapex::param::ParameterDescription("Number of classes to learn."),
                  0, 100, 2, 1),
                 std::bind(&DecisionTreeTrainer::updatePriors, this));;

    addParameter(csapex::param::ParameterFactory::declareFileOutputPath
                 ("file", "dtree.yaml"));

    addParameter(csapex::param::ParameterFactory::declareRange<int>
                 ("max depth",
                  csapex::param::ParameterDescription("The maximum possible depth of the tree. \n"
                                              "That is the training algorithms attempts to split a node while its depth is less than max_depth. \n"
                                              "The actual depth may be smaller if the other termination criteria are met \n"
                                              "(see the outline of the training procedure in the beginning of the section), and/or if the tree is pruned."),
                  1, 64, 8, 1));;
    addParameter(csapex::param::ParameterFactory::declareRange<int>
                 ("min sample count",
                  csapex::param::ParameterDescription("If the number of samples in a node is less than this parameter then the node will not be split."),
                  0, 64, 10, 1));
    addParameter(csapex::param::ParameterFactory::declareRange<double>
                 ("regression accuracy",
                  csapex::param::ParameterDescription("Termination criteria for regression trees. \n"
                                              "If all absolute differences between an estimated value in a node and values of train samples in this node \n"
                                              "are less than this parameter then the node will not be split."),
                  0.0, 255.0, 0.0, 0.01));;
    addParameter(csapex::param::ParameterFactory::declareBool
                 ("use surrogates",
                  csapex::param::ParameterDescription("If true then surrogate splits will be built. \n"
                                              "These splits allow to work with missing data and compute variable importance correctly."),
                  true));;
    addParameter(csapex::param::ParameterFactory::declareRange<int>
                 ("max categories",
                  csapex::param::ParameterDescription("Cluster possible values of a categorical variable into K < max_categories clusters to find a suboptimal split. \n"
                                              "If a discrete variable, on which the training procedure tries to make a split, \n"
                                              "takes more than max_categories values, the precise best subset estimation may take a very long time \n"
                                              "because the algorithm is exponential. \n"
                                              "Instead, many decision trees engines (including ML) try to find sub-optimal split in this case by clustering \n"
                                              "all the samples into max_categories clusters that is some categories are merged together. \n"
                                              "The clustering is applied only in n>2-class classification problems for categorical variables \n"
                                              "with N > max_categories possible values. \n"
                                              "In case of regression and 2-class classification the optimal split can be found efficiently \n"
                                              "without employing clustering, thus the parameter is not used in these cases."),
                  0, 100, 15, 1));;
    addParameter(csapex::param::ParameterFactory::declareRange<int>
                 ("cv folds",
                  csapex::param::ParameterDescription("If cv_folds > 1 then prune a tree with K-fold cross-validation where K is equal to cv_folds."),
                  0, 100, 10, 1));;
    addParameter(csapex::param::ParameterFactory::declareBool
                 ("use 1se rule",
                  csapex::param::ParameterDescription("If true then a pruning will be harsher.\n"
                                              "This will make a tree more compact and more resistant to the training data \n"
                                              "noise but a bit less accurate."),
                  true));;
    addParameter(csapex::param::ParameterFactory::declareBool
                 ("truncate pruned tree",
                  csapex::param::ParameterDescription("If true then pruned branches are physically removed from the tree. \n"
                                              "Otherwise they are retained and it is possible to get results from the \n"
                                              "original unpruned (or pruned less aggressively) tree by decreasing CvDTree::pruned_tree_idx parameter."),
                  true));;
}

void DecisionTreeTrainer::updatePriors()
{
    int categories = readParameter<int>("classes");

    if(categories != categories_) {
        if(categories > categories_) {
            for(int c = categories_; c < categories; ++c) {
                std::stringstream name;
                name << "~priors/" << c;
                csapex::param::Parameter::Ptr p = csapex::param::ParameterFactory::declareRange<double>(name.str(), 0.0, 50.0, 1.0, 0.01);
                priors_params_.push_back(p);
                addTemporaryParameter(p, std::bind(&DecisionTreeTrainer::udpatePriorValues, this));
            }
        } else {
            for(int c = categories_-1; c >= categories; --c) {
                removeTemporaryParameter(priors_params_[c]);
                priors_params_.pop_back();
            }
        }
        categories_ = categories;
    }
}

void DecisionTreeTrainer::udpatePriorValues()
{
    if(priors_.size() != priors_params_.size()) {
        priors_.resize(priors_params_.size());
    }

    for(std::size_t i = 0, total = priors_.size(); i < total; ++i) {
        priors_[i] = priors_params_[i]->as<double>();
    }
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

    CvDTreeParams params( readParameter<int>("max depth"),
                          readParameter<int>("min sample count"),
                          readParameter<double>("regression accuracy"),
                          readParameter<bool>("use surrogates"),
                          readParameter<int>("max categories"),
                          readParameter<int>("cv folds"),
                          readParameter<bool>("use 1se rule"),
                          readParameter<bool>("truncate pruned tree"),
                          priors_.data());

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

