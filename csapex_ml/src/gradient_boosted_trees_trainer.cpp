#include <opencv2/opencv.hpp>

#if CV_MAJOR_VERSION == 2
/// HEADER
#include "gradient_boosted_trees_trainer.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>

CSAPEX_REGISTER_CLASS(csapex::GradientBoostedTreesTrainer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

GradientBoostedTreesTrainer::GradientBoostedTreesTrainer() :
    classes_(0)
{
}

void GradientBoostedTreesTrainer::setupParameters(Parameterizable &parameters)
{
    CollectionNode<FeaturesMessage>::setupParameters(parameters);

    /// output path
    parameters.addParameter(param::ParameterFactory::declareFileOutputPath("path", "", "*.yaml"),
                            path_);

    /// ensemble specific parameters
    parameters.addParameter(param::ParameterFactory::declareRange<int>
                            ("gb/weak_count",
                             param::ParameterDescription("Count of trees in the ensemble."),
                             1, 4096, params_.weak_count, 1),
                            params_.weak_count);
    std::map<std::string, int> loss_function_type =
    {
        {"ABSOLUTE_LOSS", CvGBTrees::ABSOLUTE_LOSS},
        {"HUBER_LOSS", CvGBTrees::HUBER_LOSS},
        {"DEVIANCE_LOSS", CvGBTrees::DEVIANCE_LOSS}
    };
    parameters.addParameter(param::ParameterFactory::declareParameterSet
                            ("gb/loss_function_type",
                             param::ParameterDescription("Loss function used for ensemble training."),
                             loss_function_type,
                             (int) CvGBTrees::ABSOLUTE_LOSS));
    parameters.addParameter(param::ParameterFactory::declareRange<double>
                            ("gb/subsample_portion",
                             param::ParameterDescription("Portion of whole training set used for \n"
                                                         "every single tree training. \n"
                                                         "Subsample_portion value is in (0.0, 1.0]. \n"
                                                         "Subsample_portion == 1.0 when whole dataset is"
                                                         "used on each step. Count of sample used on each"
                                                         "step is computed as."),
                             0.01, 1.0, params_.subsample_portion, 0.01),
                            subsample_portion_);
    parameters.addParameter(param::ParameterFactory::declareRange<double>
                            ("gb/shrinkage",
                             param::ParameterDescription("A regularization parameter.\n"
                                                         "Each tree prediction is multiplied on shrinkage value."),
                             0.01, 1.0, params_.shrinkage, 0.01),
                            shrinkage_);

    /// tree specific parameters
    parameters.addParameter(param::ParameterFactory::declareRange<int>
                            ("dtree/classes",
                             param::ParameterDescription("Number of classes to learn."),
                             2, 100, 2, 1),
                            std::bind(&GradientBoostedTreesTrainer::updatePriors, this));;

    parameters.addParameter(param::ParameterFactory::declareRange<int>
                            ("dtree/max_depth",
                             param::ParameterDescription("The maximum possible depth of the tree. \n"
                                                         "That is the training algorithms attempts to split a node while its depth is less than max_depth. \n"
                                                         "The actual depth may be smaller if the other termination criteria are met \n"
                                                         "(see the outline of the training procedure in the beginning of the section), and/or if the tree is pruned."),
                             1, 128, params_.max_depth, 1),
                            params_.max_depth);;
    parameters.addParameter(param::ParameterFactory::declareRange<int>
                            ("dtree/min_sample_count",
                             param::ParameterDescription("If the number of samples in a node is less than this parameter then the node will not be split."),
                             0, 128, params_.min_sample_count, 1),
                            params_.min_sample_count);
    parameters.addParameter(param::ParameterFactory::declareRange<double>
                            ("dtree/regression_accuracy",
                             param::ParameterDescription("Termination criteria for regression trees. \n"
                                                         "If all absolute differences between an estimated value in a node and values of train samples in this node \n"
                                                         "are less than this parameter then the node will not be split."),
                             0.0, 255.0, params_.regression_accuracy, 0.01),
                            regression_accuracy_);;
    parameters.addParameter(param::ParameterFactory::declareBool
                            ("use surrogates",
                             param::ParameterDescription("If true then surrogate splits will be built. \n"
                                                         "These splits allow to work with missing data and compute variable importance correctly."),
                             false),
                            params_.use_surrogates);;
    parameters.addParameter(param::ParameterFactory::declareRange<int>
                            ("dtree/max_categories",
                             param::ParameterDescription("Cluster possible values of a categorical variable into K < max_categories clusters to find a suboptimal split. \n"
                                                         "If a discrete variable, on which the training procedure tries to make a split, \n"
                                                         "takes more than max_categories values, the precise best subset estimation may take a very long time \n"
                                                         "because the algorithm is exponential. \n"
                                                         "Instead, many decision trees engines (including ML) try to find sub-optimal split in this case by clustering \n"
                                                         "all the samples into max_categories clusters that is some categories are merged together. \n"
                                                         "The clustering is applied only in n>2-class classification problems for categorical variables \n"
                                                         "with N > max_categories possible values. \n"
                                                         "In case of regression and 2-class classification the optimal split can be found efficiently \n"
                                                         "without employing clustering, thus the parameter is not used in these cases."),
                             0, 100, params_.max_categories, 1),
                            params_.max_categories);;
    parameters.addParameter(param::ParameterFactory::declareRange<int>
                            ("dtree/cv_folds",
                             param::ParameterDescription("If cv_folds > 1 then prune a tree with K-fold cross-validation where K is equal to cv_folds."),
                             0, 100, params_.cv_folds, 1),
                            params_.cv_folds);;
    parameters.addParameter(param::ParameterFactory::declareBool
                            ("dtree/use_1se_rule",
                             param::ParameterDescription("If true then a pruning will be harsher.\n"
                                                         "This will make a tree more compact and more resistant to the training data \n"
                                                         "noise but a bit less accurate."),
                             params_.use_1se_rule),
                            params_.use_1se_rule);;
    parameters.addParameter(param::ParameterFactory::declareBool
                            ("dtree/truncate_pruned_tree",
                             param::ParameterDescription("If true then pruned branches are physically removed from the tree. \n"
                                                         "Otherwise they are retained and it is possible to get results from the \n"
                                                         "original unpruned (or pruned less aggressively) tree by decreasing CvDTree::pruned_tree_idx parameter."),
                             params_.truncate_pruned_tree),
                            params_.truncate_pruned_tree);;


}

bool GradientBoostedTreesTrainer::processCollection(std::vector<FeaturesMessage> &collection)
{
    int tflag = CV_ROW_SAMPLE;
    FeaturesMessage& first_feature = collection[0];
    std::size_t feature_length = first_feature.value.size();

    cv::Mat train_data(collection.size(), feature_length, CV_32FC1);
    cv::Mat missing(collection.size(), feature_length, CV_8UC1, cv::Scalar(0));

    cv::Mat responses(collection.size(), 1, CV_32SC1);


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

//    params_.priors = priors_.data();
    params_.subsample_portion = subsample_portion_;
    params_.shrinkage = shrinkage_;
    params_.regression_accuracy = regression_accuracy_;

    cv::Mat var_type( train_data.cols + 1, 1, CV_8U, CV_VAR_NUMERICAL);

    cv::GradientBoostingTrees trees;
    std::cout << "[GBTrees]: Started training with " << train_data.rows << " samples!" << std::endl;
    if(trees.train(train_data, tflag, responses, cv::Mat(), cv::Mat(), var_type, missing, params_)) {
        trees.save(path_.c_str());
        std::cout << "[GBTrees]: Finished training!" << std::endl;
    } else {
        return false;
    }

    return true;
}

void GradientBoostedTreesTrainer::updatePriors()
{
    int classes = readParameter<int>("dtree/classes");

    awarn << "Due to segfault during training, priors are currently not used" << std::endl;

    if(classes != classes_) {
        if(classes > classes_) {
            for(int c = classes_; c < classes; ++c) {
                std::stringstream name;
                name << "~priors/" << c;
                param::Parameter::Ptr p = csapex::param::ParameterFactory::declareRange<double>(name.str(), 0.0, 50.0, 1.0, 0.01);
                priors_params_.push_back(p);
                addTemporaryParameter(p, std::bind(&GradientBoostedTreesTrainer::udpatePriorValues, this));
            }
        } else {
            for(int c = classes_-1; c >= classes; --c) {
                removeTemporaryParameter(priors_params_[c]);
                priors_params_.pop_back();
            }
        }
        classes_ = classes;
    }
}

void GradientBoostedTreesTrainer::udpatePriorValues()
{
    if(priors_.size() != priors_params_.size()) {
        priors_.resize(priors_params_.size());
    }

    for(std::size_t i = 0, total = priors_.size(); i < total; ++i) {
        priors_[i] = priors_params_[i]->as<double>();
    }
}
#else
#warning Gradient boosted trees are not supported in OpenCV 3.
#endif

