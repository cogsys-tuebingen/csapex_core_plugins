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
    parameters.addParameter(csapex::param::factory::declareBool("classes/one-vs-all", false),
                            one_vs_all_);

    parameters.addParameter(param::factory::declareBool
                            ("classes/balance",
                             param::ParameterDescription("Use the same amount of samples per class."),
                             false),
                             balance_);



    /// tree specific parameters
    parameters.addParameter(csapex::param::factory::declareRange<int>
                           ("dforest/classes",
                            csapex::param::ParameterDescription("Number of classes to learn."),
                            0, 100, 2, 1),
                            std::bind(&DecisionTreeForestTrainer::updatePriors, this));;

    parameters.addParameter(csapex::param::factory::declareFileOutputPath
                           ("dforest/file", "dforest.yaml"),
                            path_);

    parameters.addParameter(csapex::param::factory::declareRange<int>
                           ("max depth",
                              csapex::param::ParameterDescription("The maximum possible depth of the tree. \n"
                                                                  "That is the training algorithms attempts to split a node while its depth is less than max_depth. \n"
                                                                  "The actual depth may be smaller if the other termination criteria are met \n"
                                                                  "(see the outline of the training procedure in the beginning of the section), and/or if the tree is pruned."),
                            1, 64, 8, 1),
                            max_depth_);;
    parameters.addParameter(csapex::param::factory::declareRange<int>
                           ("dforest/min_sample_count",
                            csapex::param::ParameterDescription("If the number of samples in a node is less than this parameter then the node will not be split."),
                            0, 64, 10, 1),
                            min_sample_count_);
    parameters.addParameter(csapex::param::factory::declareRange<double>
                           ("dforest/regression_accuracy",
                            csapex::param::ParameterDescription("Termination criteria for regression trees. \n"
                                                                "If all absolute differences between an estimated value in a node and values of train samples in this node \n"
                                                                "are less than this parameter then the node will not be split."),
                            0.0, 255.0, 0.0, 0.01),
                            regression_accuracy_);;
    parameters.addParameter(csapex::param::factory::declareBool
                           ("dforest/use_surrogates",
                            csapex::param::ParameterDescription("If true then surrogate splits will be built. \n"
                                                                "These splits allow to work with missing data and compute variable importance correctly."),
                            true),
                            use_surrogates_);;
    parameters.addParameter(csapex::param::factory::declareRange<int>
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
    parameters.addParameter(csapex::param::factory::declareRange<int>
                           ("dforest/cv_folds",
                            csapex::param::ParameterDescription("If cv_folds > 1 then prune a tree with K-fold cross-validation where K is equal to cv_folds."),
                            0, 100, 10, 1),
                            cv_folds_);;
    parameters.addParameter(csapex::param::factory::declareBool
                           ("dforest/use_1se_rule",
                            csapex::param::ParameterDescription("If true then a pruning will be harsher.\n"
                                                                "This will make a tree more compact and more resistant to the training data \n"
                                                                "noise but a bit less accurate."),
                            true),
                            use_1se_rule_);;
    parameters.addParameter(csapex::param::factory::declareBool
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
                csapex::param::Parameter::Ptr p = csapex::param::factory::declareRange<double>(name.str(), 0.0, 50.0, 1.0, 0.01);
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
    std::size_t step = collection.front().value.size();
    std::map<int, std::vector<std::size_t>> indices_by_label;

    for(std::size_t i = 0 ; i < collection.size() ; ++i) {
        const FeaturesMessage &fm = collection[i];
        if(fm.value.size() != step)
            throw std::runtime_error("All descriptors must have the same length!");
        indices_by_label[fm.classification].push_back(i);
    }


    std::vector<int> dtree_labels;
    cv::FileStorage fs(path_, cv::FileStorage::WRITE);
    const static std::string prefix = "dtree_";

#if CV_MAJOR_VERSION == 2
    int tflag = CV_ROW_SAMPLE;
#elif CV_MAJOR_VERSION == 3
//    int tflag = cv::ml::ROW_SAMPLE;
#endif


    if(indices_by_label.size() == 1) {
        throw std::runtime_error("At least 2 classes are required!");
    }



    if(one_vs_all_) {
        std::vector<std::size_t> num_of_neg(indices_by_label.size());

        auto it = indices_by_label.begin();
        for(std::size_t i = 0 ; i < indices_by_label.size() ; ++i) {
            num_of_neg[i] = it->second.size() /(indices_by_label.size() - 1);
            auto it2 = indices_by_label.begin();
            for(std::size_t j = 0 ; j < indices_by_label.size() ; ++j) {
                if(i != j){
                    if(num_of_neg[i] > it2->second.size())
                    {
                        num_of_neg[i] = it2->second.size() /(indices_by_label.size() - 1);
                    }
                }
                ++it2;
            }
            ++it;
        }

        /// iterate samples
        for(std::size_t i = 0 ; i < indices_by_label.size() ; ++i) {
            /// this is the current goal class
            auto it = indices_by_label.begin();
            std::advance(it, i);

            std::vector<std::size_t> neg_indices;
            /// now we gather negative samples, which is basically collecting all the other classes
            for(const auto &entry : indices_by_label) {
                if(entry.first == it->first)
                    continue;

                if(balance_){
                    std::vector<std::size_t> shuffled = rand_vec_.newPermutation(entry.second.size());
                    for(std::size_t n = 0; n < num_of_neg[i]; ++ n){
                        neg_indices.push_back(entry.second[shuffled[n]]);
                    }
                }
                else{
                    neg_indices.insert(neg_indices.end(),
                                       entry.second.begin(),
                                       entry.second.end());
                }
            }

            /// gather data
            const std::vector<std::size_t> &pos_indices = it->second;
            const std::size_t sample_size = pos_indices.size() + neg_indices.size();

            cv::Mat samples(sample_size, step, CV_32FC1, cv::Scalar());
            cv::Mat labels(sample_size, 1, CV_32SC1, cv::Scalar());
            cv::Mat missing(collection.size(), step, CV_8UC1, cv::Scalar());

            cv::Mat neg_samples = samples.rowRange(0, neg_indices.size());
            cv::Mat pos_samples = samples.rowRange(neg_indices.size(), sample_size);
            cv::Mat neg_labels = labels.rowRange(0, neg_indices.size());
            cv::Mat pos_labels = labels.rowRange(neg_indices.size(), sample_size);
            cv::Mat neg_missing = missing.rowRange(0, neg_indices.size());
            cv::Mat pos_missing = missing.rowRange(neg_indices.size(), sample_size);
            for(int i = 0 ; i < neg_samples.rows; ++i) {
                const std::vector<float> &data = collection.at(neg_indices.at(i)).value;
                neg_labels.at<float>(i) = NEGATIVE;
                for(std::size_t j = 0 ; j < step ; ++j) {
                    const float val = data.at(j);
                    if(std::abs(val) >= FLT_MAX * 0.5f) {
                        neg_missing.at<uchar>(i,j) = 1;
                    } else {
                        neg_samples.at<float>(i,j) = val;
                    }
                }
            }
            for(int i = 0 ; i < pos_samples.rows ; ++i) {
                const std::vector<float> &data = collection.at(pos_indices.at(i)).value;
                pos_labels.at<float>(i) = POSITIVE;
                for(std::size_t j = 0 ; j < step ; ++j) {
                    const float val = data.at(j);
                    if(std::abs(val) >= FLT_MAX * 0.5f) {
                        pos_missing.at<uchar>(i,j) = 1;
                    } else {
                        pos_samples.at<float>(i,j) = val;
                    }
                }
            }

#if CV_MAJOR_VERSION == 2
            CvDTreeParams params ( max_depth_,
                                   min_sample_count_,
                                   regression_accuracy_,
                                   use_surrogates_,
                                   max_categories_,
                                   cv_folds_,
                                   use_1se_rule_,
                                   truncate_pruned_tree_,
                                   priors_.data());
            cv::Mat var_type( samples.cols + 1, 1, CV_8U, CV_VAR_NUMERICAL);
            cv::DecisionTree dtree;
            std::cout << "[DecisionTree]: Started training with " << samples.rows << " samples!" << std::endl;
            if(dtree.train(samples, tflag, labels, cv::Mat(), cv::Mat(), var_type, missing, params)) {
                std::string label = prefix + std::to_string(it->first);
                std::cout << "Finished training for tree '" << label << "'!" << std::endl;
                dtree.write(fs.fs, label.c_str());
                dtree_labels.push_back(it->first);
            } else {
                return false;
            }
#elif CV_MAJOR_VERSION == 3
        throw std::runtime_error("Not implemented yet!");
#endif
        }
    } else {
        if(indices_by_label.find(NEGATIVE) == indices_by_label.end()) {
            throw std::runtime_error("Need common negative examples labelled with -1");
        }



        if(indices_by_label.size() != priors_.size()) {
            throw std::runtime_error("Make sure to set the right amount of classes / update the priors!");
        }

        std::vector<std::size_t> neg_indices_org = indices_by_label[NEGATIVE];
        indices_by_label.erase(NEGATIVE);

        /// iterate samples
        for(std::size_t i = 0 ; i < indices_by_label.size() ; ++i) {
            auto it = indices_by_label.begin();
            std::advance(it, i);
            std::vector<std::size_t> neg_indices = neg_indices_org;
            if(balance_){
                std::vector<std::size_t> shuffled = rand_vec_.newPermutation(it->second.size());
                neg_indices.resize(shuffled.size());
                for(std::size_t i = 0; i < neg_indices.size(); ++i)
                {
                    neg_indices[i] = neg_indices_org[shuffled[i]];
                }
            }
            /// gather data
            const std::vector<std::size_t> &pos_indices = it->second;
            const std::size_t sample_size = pos_indices.size() + neg_indices.size();

            cv::Mat samples(sample_size, step, CV_32FC1, cv::Scalar());
            cv::Mat labels(sample_size, 1, CV_32SC1, cv::Scalar());
            cv::Mat missing(collection.size(), step, CV_8UC1, cv::Scalar());

            cv::Mat neg_samples = samples.rowRange(0, neg_indices.size());
            cv::Mat pos_samples = samples.rowRange(neg_indices.size(), sample_size);
            cv::Mat neg_labels = labels.rowRange(0, neg_indices.size());
            cv::Mat pos_labels = labels.rowRange(neg_indices.size(), sample_size);
            cv::Mat neg_missing = missing.rowRange(0, neg_indices.size());
            cv::Mat pos_missing = missing.rowRange(neg_indices.size(), sample_size);
            for(int i = 0 ; i < neg_samples.rows; ++i) {
                const std::vector<float> &data = collection.at(neg_indices.at(i)).value;
                neg_labels.at<float>(i) = NEGATIVE;
                for(std::size_t j = 0 ; j < step ; ++j) {
                    const float val = data.at(j);
                    if(std::abs(val) >= FLT_MAX * 0.5f) {
                        neg_missing.at<uchar>(i,j) = 1;
                    } else {
                        neg_samples.at<float>(i,j) = val;
                    }                }
            }
            for(int i = 0 ; i < pos_samples.rows ; ++i) {
                const std::vector<float> &data = collection.at(pos_indices.at(i)).value;
                pos_labels.at<float>(i) = POSITIVE;
                for(std::size_t j = 0 ; j < step ; ++j) {
                    const float val = data.at(j);
                    if(std::abs(val) >= FLT_MAX * 0.5f) {
                        pos_missing.at<uchar>(i,j) = 1;
                    } else {
                        pos_samples.at<float>(i,j) = val;
                    }
                }
            }

#if CV_MAJOR_VERSION == 2
            CvDTreeParams params ( max_depth_,
                                   min_sample_count_,
                                   regression_accuracy_,
                                   use_surrogates_,
                                   max_categories_,
                                   cv_folds_,
                                   use_1se_rule_,
                                   truncate_pruned_tree_,
                                   priors_.data());
            cv::Mat var_type( samples.cols + 1, 1, CV_8U, CV_VAR_NUMERICAL);
            cv::DecisionTree dtree;
            std::cout << "[DecisionTree]: Started training with " << samples.rows << " samples!" << std::endl;
            if(dtree.train(samples, tflag, labels, cv::Mat(), cv::Mat(), var_type, missing, params)) {
                std::string label = prefix + std::to_string(it->first);
                std::cout << "Finished training for tree '" << label << "'!" << std::endl;
                dtree.write(fs.fs, label.c_str());
                dtree_labels.push_back(it->first);
            } else {
                return false;
            }
#elif CV_MAJOR_VERSION == 3
        throw std::runtime_error("Not implemented yet!");
#endif
        }
    }
    fs << "labels" << dtree_labels;
    fs.release();
    return true;
}
