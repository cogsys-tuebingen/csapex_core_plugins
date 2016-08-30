/// HEADER
#include "svm_ensemble_trainer.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>

#include "extended_svm.hpp"

CSAPEX_REGISTER_CLASS(csapex::SVMEnsembleTrainer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

SVMEnsembleTrainer::SVMEnsembleTrainer():
    rand_vec_(2)
{
}

void SVMEnsembleTrainer::setup(NodeModifier &modifier)
{
    CollectionNode<connection_types::FeaturesMessage>::setup(modifier);
}

void SVMEnsembleTrainer::setupParameters(Parameterizable& parameters)
{
    CollectionNode<connection_types::FeaturesMessage>::setupParameters(parameters);

    parameters.addParameter(param::ParameterFactory::declareFileOutputPath("svm/path",
                                                                           param::ParameterDescription("File to write svm to."),
                                                                           "",
                                                                           "*.yaml *.tar.gz"),
                            path_);

    parameters.addParameter(param::ParameterFactory::declareBool("svm/save_coeffs",
                                                                 param::ParameterDescription("Save precomputed vector for HOG."),
                                                                 false),
                            save_for_hog_);


    std::map<std::string, int> kernel_types = {
        {"LINEAR", cv::SVM::LINEAR},
        {"POLY", cv::SVM::POLY},
        {"RBF", cv::SVM::RBF},
        {"SIGMOID", cv::SVM::SIGMOID}
    };

    parameters.addParameter(param::ParameterFactory::declareParameterSet("svm/kernel_type",
                                                                         csapex::param::ParameterDescription("Kernel type to be trained."),
                                                                         kernel_types,
                                                                         (int) cv::SVM::RBF),
                            svm_params_.kernel_type);

    std::map<std::string, int> svm_types = {
        {"C_SVC", cv::SVM::C_SVC},
        {"NU_SVC", cv::SVM::NU_SVC},
        {"ONE_CLASS", cv::SVM::ONE_CLASS},
        {"EPS_SVR", cv::SVM::EPS_SVR},
        {"NU_SVR", cv::SVM::NU_SVR}
    };

    parameters.addParameter(param::ParameterFactory::declareParameterSet("svm type",
                                                                         csapex::param::ParameterDescription("SVM type to be trained."),
                                                                         svm_types,
                                                                         (int) cv::SVM::EPS_SVR),
                            svm_params_.svm_type);


    std::function<bool()> deg_cond    = [this]() {
        return svm_params_.kernel_type == cv::SVM::POLY;};

    std::function<bool()> gamma_cond  = [this]() {
        return svm_params_.kernel_type == cv::SVM::POLY ||
                svm_params_.kernel_type == cv::SVM::RBF ||
                svm_params_.kernel_type == cv::SVM::SIGMOID;};

    std::function<bool()> coeff0_cond = [this]() {
        return svm_params_.kernel_type == cv::SVM::POLY ||
                svm_params_.kernel_type == cv::SVM::SIGMOID;};

    std::function<bool()> c_cond = [this]() {
        return svm_params_.svm_type == cv::SVM::C_SVC ||
                svm_params_.svm_type == cv::SVM::EPS_SVR ||
                svm_params_.svm_type == cv::SVM::NU_SVR;};

    std::function<bool()> nu_cond = [this]() {
        return svm_params_.svm_type == cv::SVM::ONE_CLASS ||
                svm_params_.svm_type == cv::SVM::NU_SVR ||
                svm_params_.svm_type == cv::SVM::EPS_SVR;};

    std::function<bool()> p_cond = [this]() {
        return svm_params_.svm_type == cv::SVM::EPS_SVR;};

    std::function<bool()> one_class_cond = [this]() {
        return svm_params_.svm_type != cv::SVM::ONE_CLASS;};


    parameters.addConditionalParameter(param::ParameterFactory::declareRange<double>("degree", 0.0, 9.0, 3.0, 1.0),
                                       deg_cond,
                                       svm_params_.degree);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange<double>("gamma", 0.0, 10.0, 0.0, 0.01),
                                       gamma_cond,
                                       svm_params_.gamma);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange<double>("coef0", -10.0, 10.0, 0.0, 0.01),
                                       coeff0_cond,
                                       svm_params_.coef0);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange<double>("C", 0.0, 10.0, 0.01, 0.01),
                                       c_cond,
                                       svm_params_.C);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange<double>("nu", 0.0, 1.0, 0.5, 0.01),
                                       nu_cond,
                                       svm_params_.nu);
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareRange<double>("p", 0.0, 1.0, 0.1, 0.01),
                                       p_cond,
                                       svm_params_.p);
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareBool("one-vs-all", false),
                                       one_class_cond,
                                       one_vs_all_);

    parameters.addParameter(param::ParameterFactory::declareBool
                            ("balance",
                             param::ParameterDescription("Use the same amount of samples per class."),
                             false),
                            balance_);

}

bool SVMEnsembleTrainer::processCollection(std::vector<FeaturesMessage> &collection)
{
    std::size_t step = collection.front().value.size();
    std::map<int, std::vector<std::size_t>> indices_by_label;

    for(std::size_t i = 0 ; i < collection.size() ; ++i) {
        const FeaturesMessage &fm = collection[i];
        if(fm.value.size() != step)
            throw std::runtime_error("All descriptors must have the same length!");
        indices_by_label[fm.classification].push_back(i);
    }


    std::vector<int> svm_labels;
    cv::FileStorage fs(path_, cv::FileStorage::WRITE);
    const static std::string prefix = "svm_";
    if(svm_params_.svm_type != cv::SVM::ONE_CLASS) {
        if(indices_by_label.size() == 1) {
            throw std::runtime_error("Multi class SVMs require multiple classes!");
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

                ExtendedSVM svm;
                /// gather data
                const std::vector<std::size_t> &pos_indices = it->second;
                const std::size_t sample_size = pos_indices.size() + neg_indices.size();

                cv::Mat samples(sample_size, step, CV_32FC1, cv::Scalar());
                cv::Mat labels(sample_size, 1, CV_32FC1, cv::Scalar());

                cv::Mat neg_samples = samples.rowRange(0, neg_indices.size());
                cv::Mat pos_samples = samples.rowRange(neg_indices.size(), sample_size);
                cv::Mat neg_labels = labels.rowRange(0, neg_indices.size());
                cv::Mat pos_labels = labels.rowRange(neg_indices.size(), sample_size);
                for(int i = 0 ; i < neg_samples.rows; ++i) {
                    const std::vector<float> &data = collection.at(neg_indices.at(i)).value;
                    neg_labels.at<float>(i) = NEGATIVE;
                    for(std::size_t j = 0 ; j < step ; ++j) {
                        neg_samples.at<float>(i,j) = data.at(j);
                    }
                }
                for(int i = 0 ; i < pos_samples.rows ; ++i) {
                    const std::vector<float> &data = collection.at(pos_indices.at(i)).value;
                    pos_labels.at<float>(i) = POSITIVE;
                    for(std::size_t j = 0 ; j < step ; ++j) {
                        pos_samples.at<float>(i,j) = data.at(j);
                    }
                }
                cv::SVMParams params = svm_params_;
                if(params.gamma == 0) {
                    params.gamma = 1.0 / labels.rows;
                }

                /// train the svm
                std::cout << "Started training for '" << it->first << std::endl;
                if(svm.train(samples, labels, cv::Mat(), cv::Mat(), params)) {
                    std::cout << "Finished training for '" << it->first << "'!" << std::endl;
                    std::string label = prefix + std::to_string(it->first);
                    svm.write(fs.fs, label.c_str());
                    svm_labels.push_back(it->first);
                } else {
                    return false;
                }
            }
        } else {
            if(indices_by_label.find(NEGATIVE) == indices_by_label.end()) {
                throw std::runtime_error("Need negative examples labelled with -1");
            }
            if(indices_by_label.size() == 1) {
                throw std::runtime_error("Multi class SVMs require multiple classes!");
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
                ExtendedSVM svm;
                /// gather data
                const std::vector<std::size_t> &pos_indices = it->second;
                const std::size_t sample_size = pos_indices.size() + neg_indices.size();

                cv::Mat samples(sample_size, step, CV_32FC1, cv::Scalar());
                cv::Mat labels(sample_size, 1, CV_32FC1, cv::Scalar());

                cv::Mat neg_samples = samples.rowRange(0, neg_indices.size());
                cv::Mat pos_samples = samples.rowRange(neg_indices.size(), sample_size);
                cv::Mat neg_labels = labels.rowRange(0, neg_indices.size());
                cv::Mat pos_labels = labels.rowRange(neg_indices.size(), sample_size);
                for(int i = 0 ; i < neg_samples.rows; ++i) {
                    const std::vector<float> &data = collection.at(neg_indices.at(i)).value;
                    neg_labels.at<float>(i) = NEGATIVE;
                    for(std::size_t j = 0 ; j < step ; ++j) {
                        neg_samples.at<float>(i,j) = data.at(j);
                    }
                }
                for(int i = 0 ; i < pos_samples.rows ; ++i) {
                    const std::vector<float> &data = collection.at(pos_indices.at(i)).value;
                    pos_labels.at<float>(i) = POSITIVE;
                    for(std::size_t j = 0 ; j < step ; ++j) {
                        pos_samples.at<float>(i,j) = data.at(j);
                    }
                }
                cv::SVMParams params = svm_params_;
                if(params.gamma == 0) {
                    params.gamma = 1.0 / labels.rows;
                }

                /// train the svm
                std::cout << "[SVMEnsemble]: Started training for svm #"
                          << it->first << " with " << samples.rows << std::endl;
                if(svm.train(samples, labels, cv::Mat(), cv::Mat(), params)) {
                    std::string label = prefix + std::to_string(it->first);
                    svm.write(fs.fs, label.c_str());
                    svm_labels.push_back(it->first);
                    std::cout << "[SVMEnsemble]: Finished training for svm #"
                              << it->first << "!" << std::endl;
                } else {
                    return false;
                }
            }
        }
    } else {
        if(indices_by_label.find(NEGATIVE) != indices_by_label.end()) {
            indices_by_label.erase(NEGATIVE);
        }
        /// iterate samples
        /// mixed classes?
        for(std::size_t i = 0 ; i < indices_by_label.size() ; ++i) {
            auto it = indices_by_label.begin();
            std::advance(it, i);

            /// allocate svm
            ExtendedSVM svm;

            /// gather data
            const std::vector<std::size_t> &indices = it->second;
            const std::size_t sample_size = indices.size();

            cv::Mat samples(sample_size, step, CV_32FC1, cv::Scalar());
            cv::Mat labels(sample_size, 1, CV_32FC1, cv::Scalar());

            for(int i = 0 ; i < samples.rows ; ++i) {
                const std::vector<float> &data = collection.at(indices.at(i)).value;
                labels.at<float>(i) = POSITIVE;
                for(std::size_t j = 0 ; j < step ; ++j) {
                    samples.at<float>(i,j) = data.at(j);
                }
            }
            cv::SVMParams params = svm_params_;
            if(params.gamma == 0) {
                params.gamma = 1.0 / labels.rows;
            }

            /// train the svm
            std::cout << "[SVMEnsemble]: Started training for svm #"
                      << it->first << " with " << samples.rows << std::endl;
            if(svm.train(samples, labels, cv::Mat(), cv::Mat(), params)) {
                std::string label = prefix + std::to_string(it->first);
                svm.write(fs.fs, label.c_str());
                svm_labels.push_back(it->first);
                std::cout << "[SVMEnsemble]: Finished training for svm #"
                          << it->first << "!" << std::endl;
            } else {
                return false;
            }
        }
    }
    fs << "labels" << svm_labels;
    fs.release();
    return true;
}
