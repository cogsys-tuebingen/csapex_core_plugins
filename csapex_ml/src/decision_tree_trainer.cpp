/// HEADER
#include "decision_tree_trainer.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <opencv2/ml/ml.hpp>

CSAPEX_REGISTER_CLASS(csapex::DecisionTreeTrainer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

DecisionTreeTrainer::DecisionTreeTrainer() : classes_(0)
{
}

void DecisionTreeTrainer::setup(NodeModifier& node_modifier)
{
    CollectionNode<connection_types::FeaturesMessage>::setup(node_modifier);
}

void DecisionTreeTrainer::setupParameters(Parameterizable& parameters)
{
    CollectionNode<FeaturesMessage>::setupParameters(parameters);

    parameters.addParameter(csapex::param::factory::declareRange<int>("classes", csapex::param::ParameterDescription("Number of classes to learn."), 0, 100, 2, 1),
                            std::bind(&DecisionTreeTrainer::updatePriors, this));
    ;

    parameters.addParameter(csapex::param::factory::declareFileOutputPath("file", "dtree.yaml"), path_);

    parameters.addParameter(csapex::param::factory::declareRange<int>("max depth",
                                                                      csapex::param::ParameterDescription("The maximum possible depth of the tree. \n"
                                                                                                          "That is the training algorithms attempts to split a node while "
                                                                                                          "its depth is less than max_depth. \n"
                                                                                                          "The actual depth may be smaller if the other termination "
                                                                                                          "criteria are met \n"
                                                                                                          "(see the outline of the training procedure in the beginning of "
                                                                                                          "the section), and/or if the tree is pruned."),
                                                                      1, 64, 8, 1),
                            max_depth_);
    ;
    parameters.addParameter(csapex::param::factory::declareRange<int>("min sample count",
                                                                      csapex::param::ParameterDescription("If the number of samples in a node is less than this parameter "
                                                                                                          "then the node will not be split."),
                                                                      0, 64, 10, 1),
                            min_sample_count_);
    parameters.addParameter(csapex::param::factory::declareRange<double>("regression accuracy",
                                                                         csapex::param::ParameterDescription("Termination criteria for regression trees. \n"
                                                                                                             "If all absolute differences between an estimated value in a "
                                                                                                             "node and values of train samples in this node \n"
                                                                                                             "are less than this parameter then the node will not be split."),
                                                                         0.0, 255.0, 0.0, 0.01),
                            regression_accuracy_);
    ;
    parameters.addParameter(csapex::param::factory::declareBool("use surrogates",
                                                                csapex::param::ParameterDescription("If true then surrogate splits will be built. \n"
                                                                                                    "These splits allow to work with missing data and compute "
                                                                                                    "variable importance correctly."),
                                                                true),
                            use_surrogates_);
    ;
    parameters.addParameter(csapex::param::factory::declareRange<int>("max categories",
                                                                      csapex::param::ParameterDescription("Cluster possible values of a categorical variable into K < "
                                                                                                          "max_categories clusters to find a suboptimal split. \n"
                                                                                                          "If a discrete variable, on which the training procedure tries "
                                                                                                          "to make a split, \n"
                                                                                                          "takes more than max_categories values, the precise best subset "
                                                                                                          "estimation may take a very long time \n"
                                                                                                          "because the algorithm is exponential. \n"
                                                                                                          "Instead, many decision trees engines (including ML) try to find "
                                                                                                          "sub-optimal split in this case by clustering \n"
                                                                                                          "all the samples into max_categories clusters that is some "
                                                                                                          "categories are merged together. \n"
                                                                                                          "The clustering is applied only in n>2-class classification "
                                                                                                          "problems for categorical variables \n"
                                                                                                          "with N > max_categories possible values. \n"
                                                                                                          "In case of regression and 2-class classification the optimal "
                                                                                                          "split can be found efficiently \n"
                                                                                                          "without employing clustering, thus the parameter is not used in "
                                                                                                          "these cases."),
                                                                      0, 100, 15, 1),
                            max_categories_);
    ;
    parameters.addParameter(csapex::param::factory::declareRange<int>("cv folds",
                                                                      csapex::param::ParameterDescription("If cv_folds > 1 then prune a tree with K-fold cross-validation "
                                                                                                          "where K is equal to cv_folds."),
                                                                      0, 100, 10, 1),
                            cv_folds_);
    ;
    parameters.addParameter(csapex::param::factory::declareBool("use 1se rule",
                                                                csapex::param::ParameterDescription("If true then a pruning will be harsher.\n"
                                                                                                    "This will make a tree more compact and more "
                                                                                                    "resistant to the training data \n"
                                                                                                    "noise but a bit less accurate."),
                                                                true),
                            use_1se_rule_);
    ;
    parameters.addParameter(csapex::param::factory::declareBool("truncate pruned tree",
                                                                csapex::param::ParameterDescription("If true then pruned branches are physically removed from the "
                                                                                                    "tree. \n"
                                                                                                    "Otherwise they are retained and it is possible to get results "
                                                                                                    "from the \n"
                                                                                                    "original unpruned (or pruned less aggressively) tree by "
                                                                                                    "decreasing CvDTree::pruned_tree_idx parameter."),
                                                                true),
                            truncate_pruned_tree_);
    ;
}

void DecisionTreeTrainer::updatePriors()
{
    int classes = readParameter<int>("classes");

    if (classes != classes_) {
        if (classes > classes_) {
            for (int c = classes_; c < classes; ++c) {
                std::stringstream name;
                name << "~priors/" << c;
                csapex::param::Parameter::Ptr p = csapex::param::factory::declareRange<double>(name.str(), 0.0, 50.0, 1.0, 0.01);
                priors_params_.push_back(p);
                addTemporaryParameter(p, std::bind(&DecisionTreeTrainer::udpatePriorValues, this));
            }
        } else {
            for (int c = classes_ - 1; c >= classes; --c) {
                removeTemporaryParameter(priors_params_[c]);
                priors_params_.pop_back();
            }
        }
        classes_ = classes;
    }
}

void DecisionTreeTrainer::udpatePriorValues()
{
    if (priors_.size() != priors_params_.size()) {
        priors_.resize(priors_params_.size());
    }

    for (std::size_t i = 0, total = priors_.size(); i < total; ++i) {
        priors_[i] = priors_params_[i]->as<double>();
    }
}

bool DecisionTreeTrainer::processCollection(std::vector<connection_types::FeaturesMessage>& collection)
{
    FeaturesMessage& first_feature = collection[0];
    std::size_t feature_length = first_feature.value.size();

    cv::Mat train_data(collection.size(), feature_length, CV_32FC1);
    cv::Mat missing(collection.size(), feature_length, CV_8UC1, cv::Scalar(0));

    cv::Mat responses(collection.size(), 1, CV_32SC1);

#if CV_MAJOR_VERSION == 2
    int tflag = CV_ROW_SAMPLE;
#elif CV_MAJOR_VERSION >= 3
    int tflag = cv::ml::ROW_SAMPLE;
#endif

    std::size_t n = collection.size();
    for (std::size_t i = 0; i < n; ++i) {
        FeaturesMessage& feature = collection[i];
        for (std::size_t j = 0; j < feature_length; ++j) {
            const float& val = feature.value[j];

            if (std::abs(val) >= FLT_MAX * 0.5f) {
                missing.at<uchar>(i, j) = 1;
            } else {
                train_data.at<float>(i, j) = val;
            }
        }

        responses.at<int>(i, 0) = feature.classification;
    }

#if CV_MAJOR_VERSION == 2
    CvDTreeParams params(max_depth_, min_sample_count_, regression_accuracy_, use_surrogates_, max_categories_, cv_folds_, use_1se_rule_, truncate_pruned_tree_, priors_.data());

    cv::Mat var_type(train_data.cols + 1, 1, CV_8U, CV_VAR_NUMERICAL);

    cv::DecisionTree dtree;
    std::cout << "[DecisionTree]: Started training with " << train_data.rows << " samples!" << std::endl;
    if (dtree.train(train_data, tflag, responses, cv::Mat(), cv::Mat(), var_type, missing, params)) {
        dtree.save(readParameter<std::string>("file").c_str());
        std::cout << "[DecisionTree]: Finished training!" << std::endl;
    } else {
        return false;
    }

#elif CV_MAJOR_VERSION >= 3
    auto dtree = cv::ml::DTrees::create();
    dtree->setMaxDepth(max_depth_);
    dtree->setMinSampleCount(min_sample_count_);
    dtree->setRegressionAccuracy(regression_accuracy_);
    dtree->setUseSurrogates(use_surrogates_);
    dtree->setMaxCategories(max_categories_);
    dtree->setCVFolds(cv_folds_);
    dtree->setUse1SERule(use_1se_rule_);
    dtree->setTruncatePrunedTree(truncate_pruned_tree_);

    cv::Mat priors(priors_);
    dtree->setPriors(priors);

    cv::Mat var_type(train_data.cols + 1, 1, CV_8U, cv::ml::VAR_NUMERICAL);

    cv::Ptr<cv::ml::TrainData> train_data_struct = cv::ml::TrainData::create(train_data, tflag, responses, cv::noArray(), cv::noArray(), cv::noArray(), var_type);

    std::cout << "[DecisionTree]: Started training with " << train_data.rows << " samples!" << std::endl;
    if (dtree->train(train_data_struct)) {
        dtree->save(readParameter<std::string>("file").c_str());
        std::cout << "[DecisionTree]: Finished training!" << std::endl;
    } else {
        return false;
    }
#endif
    return true;
}
