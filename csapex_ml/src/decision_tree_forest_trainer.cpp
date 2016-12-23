/// HEADER
#include "decision_tree_forest_trainer.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>

CSAPEX_REGISTER_CLASS(csapex::DecisionTreeForestTrainer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

DecisionTreeForestTrainer::DecisionTreeForestTrainer() :
    rand_vec_(2),
    classes_(0)
{
}

void DecisionTreeForestTrainer::setup(NodeModifier &node_modifier)
{
    CollectionNode<connection_types::FeaturesMessage>::setup(node_modifier);
}

void DecisionTreeForestTrainer::setupParameters(Parameterizable &parameters)
{
    CollectionNode<FeaturesMessage>::setupParameters(parameters);

    /// sample usage specific parameters
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("classes/one-vs-all", false),
                            one_vs_all_);

    parameters.addParameter(param::ParameterFactory::declareBool
                            ("classes/balance",
                             param::ParameterDescription("Use the same amount of samples per class."),
                             false),
                             balance_);



    /// tree specific parameters
    parameters.addParameter(csapex::param::ParameterFactory::declareRange<int>
                           ("dforest/classes",
                            csapex::param::ParameterDescription("Number of classes to learn."),
                            0, 100, 2, 1),
                            std::bind(&DecisionTreeForestTrainer::updatePriors, this));;

    parameters.addParameter(csapex::param::ParameterFactory::declareFileOutputPath
                           ("dforest/file", "dforest.yaml"),
                            path_);

    parameters.addParameter(csapex::param::ParameterFactory::declareRange<int>
                           ("max depth",
                              csapex::param::ParameterDescription("The maximum possible depth of the tree. \n"
                                                                  "That is the training algorithms attempts to split a node while its depth is less than max_depth. \n"
                                                                  "The actual depth may be smaller if the other termination criteria are met \n"
                                                                  "(see the outline of the training procedure in the beginning of the section), and/or if the tree is pruned."),
                            1, 64, 8, 1),
                            max_depth_);;
    parameters.addParameter(csapex::param::ParameterFactory::declareRange<int>
                           ("dforest/min_sample_count",
                            csapex::param::ParameterDescription("If the number of samples in a node is less than this parameter then the node will not be split."),
                            0, 64, 10, 1),
                            min_sample_count_);
    parameters.addParameter(csapex::param::ParameterFactory::declareRange<double>
                           ("dforest/regression_accuracy",
                            csapex::param::ParameterDescription("Termination criteria for regression trees. \n"
                                                                "If all absolute differences between an estimated value in a node and values of train samples in this node \n"
                                                                "are less than this parameter then the node will not be split."),
                            0.0, 255.0, 0.0, 0.01),
                            regression_accuracy_);;
    parameters.addParameter(csapex::param::ParameterFactory::declareBool
                           ("dforest/use_surrogates",
                            csapex::param::ParameterDescription("If true then surrogate splits will be built. \n"
                                                                "These splits allow to work with missing data and compute variable importance correctly."),
                            true),
                            use_surrogates_);;
    parameters.addParameter(csapex::param::ParameterFactory::declareRange<int>
                           ("dforest/max_categories",
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
                            0, 100, 15, 1),
                            max_categories_);;
    parameters.addParameter(csapex::param::ParameterFactory::declareRange<int>
                           ("dforest/cv_folds",
                            csapex::param::ParameterDescription("If cv_folds > 1 then prune a tree with K-fold cross-validation where K is equal to cv_folds."),
                            0, 100, 10, 1),
                            cv_folds_);;
    parameters.addParameter(csapex::param::ParameterFactory::declareBool
                           ("dforest/use_1se_rule",
                            csapex::param::ParameterDescription("If true then a pruning will be harsher.\n"
                                                                "This will make a tree more compact and more resistant to the training data \n"
                                                                "noise but a bit less accurate."),
                            true),
                            use_1se_rule_);;
    parameters.addParameter(csapex::param::ParameterFactory::declareBool
                           ("dforest/truncate_pruned_tree",
                             csapex::param::ParameterDescription("If true then pruned branches are physically removed from the tree. \n"
                                                                 "Otherwise they are retained and it is possible to get results from the \n"
                                                                 "original unpruned (or pruned less aggressively) tree by decreasing CvDTree::pruned_tree_idx parameter."),
                            true),
                            truncate_pruned_tree_);;
}

void DecisionTreeForestTrainer::updatePriors()
{
    int classes = readParameter<int>("dforest/classes");

    if(classes != classes_) {
        if(classes > classes_) {
            for(int c = classes_; c < classes; ++c) {
                std::stringstream name;
                name << "~priors/" << c;
                csapex::param::Parameter::Ptr p = csapex::param::ParameterFactory::declareRange<double>(name.str(), 0.0, 50.0, 1.0, 0.01);
                priors_params_.push_back(p);
                addTemporaryParameter(p, std::bind(&DecisionTreeForestTrainer::updatePriorValues, this));
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

void DecisionTreeForestTrainer::updatePriorValues()
{
    if(priors_.size() != priors_params_.size()) {
        priors_.resize(priors_params_.size());
    }

    for(std::size_t i = 0, total = priors_.size(); i < total; ++i) {
        priors_[i] = priors_params_[i]->as<double>();
    }
}


bool DecisionTreeForestTrainer::processCollection(std::vector<connection_types::FeaturesMessage> &collection)
{

    return true;
}
