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


#if CV_MAJOR_VERSION == 2
    std::map<std::string, int> kernel_types = {
        {"LINEAR", cv::SVM::LINEAR},
        {"POLY", cv::SVM::POLY},
        {"RBF", cv::SVM::RBF},
        {"SIGMOID", cv::SVM::SIGMOID}
    };
#elif CV_MAJOR_VERSION == 3
    std::map<std::string, int> kernel_types = {
        {"LINEAR", cv::ml::SVM::LINEAR},
        {"POLY", cv::ml::SVM::POLY},
        {"RBF", cv::ml::SVM::RBF},
        {"SIGMOID", cv::ml::SVM::SIGMOID}//,
        //{"CHI2", cv::ml::SVM::CHI2},
        //{"INTER", cv::ml::SVM::INTER}
    };

#endif


#if CV_MAJOR_VERSION == 2
    typedef cv::SVM SVM;
#elif CV_MAJOR_VERSION == 3
    typedef cv::ml::SVM SVM;
#endif

    parameters.addParameter(param::ParameterFactory::declareParameterSet("svm/kernel_type",
                                                                         csapex::param::ParameterDescription("Kernel type to be trained."),
                                                                         kernel_types,
                                                                         (int) SVM::RBF),
                            kernel_type_);

    std::map<std::string, int> svm_types = {
        {"C_SVC", SVM::C_SVC},
        {"NU_SVC", SVM::NU_SVC},
        {"ONE_CLASS", SVM::ONE_CLASS},
        {"EPS_SVR", SVM::EPS_SVR},
        {"NU_SVR", SVM::NU_SVR}
    };

    parameters.addParameter(param::ParameterFactory::declareParameterSet("svm type",
                                                                         csapex::param::ParameterDescription("SVM type to be trained."),
                                                                         svm_types,
                                                                         (int) SVM::EPS_SVR),
                            svm_type_);


    std::function<bool()> deg_cond    = [this]() {
        return kernel_type_ == SVM::POLY;};

    std::function<bool()> gamma_cond  = [this]() {
        return kernel_type_ == SVM::POLY ||
                kernel_type_ == SVM::RBF ||
                kernel_type_ == SVM::SIGMOID;};

    std::function<bool()> coeff0_cond = [this]() {
        return kernel_type_ == SVM::POLY ||
                kernel_type_ == SVM::SIGMOID;};

    std::function<bool()> c_cond = [this]() {
        return svm_type_ == SVM::C_SVC ||
                svm_type_ == SVM::EPS_SVR ||
                svm_type_ == SVM::NU_SVR;};

    std::function<bool()> nu_cond = [this]() {
        return svm_type_ == SVM::ONE_CLASS ||
                svm_type_ == SVM::NU_SVR ||
                svm_type_ == SVM::EPS_SVR;};

    std::function<bool()> p_cond = [this]() {
        return svm_type_ == SVM::EPS_SVR;};

    std::function<bool()> one_class_cond = [this]() {
        return svm_type_ != SVM::ONE_CLASS;};


    parameters.addConditionalParameter(param::ParameterFactory::declareRange<double>("degree", 0.0, 9.0, 3.0, 1.0),
                                       deg_cond,
                                       degree_);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange<double>("gamma", 0.0, 10.0, 0.0, 0.01),
                                       gamma_cond,
                                       gamma_);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange<double>("coef0", -10.0, 10.0, 0.0, 0.01),
                                       coeff0_cond,
                                       coef0_);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange<double>("C", 0.0, 10.0, 0.01, 0.01),
                                       c_cond,
                                       C_);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange<double>("nu", 0.0, 1.0, 0.5, 0.01),
                                       nu_cond,
                                       nu_);
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareRange<double>("p", 0.0, 1.0, 0.1, 0.01),
                                       p_cond,
                                       p_);
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
#if CV_MAJOR_VERSION == 2
    if(svm_type_ != cv::SVM::ONE_CLASS) {
#elif CV_MAJOR_VERSION == 3
    if(svm_type != cv::ml::SVM::ONE_CLASS) {
#endif
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

                if(gamma_ == 0) {
                    gamma_ = 1.0 / labels.rows;
                }

#if CV_MAJOR_VERSION == 2
                ExtendedSVM svm;

                cv::SVMParams   svm_params_;
                svm_params_.svm_type = svm_type_;
                svm_params_.kernel_type = kernel_type_;
                svm_params_.degree = degree_;
                svm_params_.gamma = gamma_;
                svm_params_.coef0 = coef0_;

                svm_params_.C = C_;
                svm_params_.nu = nu_;
                svm_params_.p = p_;
                //svm_params_.term_crit; // termination criteria

                /// train the svm
                std::cout << "Started training for '" << it->first << std::endl;
                if(svm.train(samples, labels, cv::Mat(), cv::Mat(), svm_params_)) {
                    std::cout << "Finished training for '" << it->first << "'!" << std::endl;
                    std::string label = prefix + std::to_string(it->first);
                    svm.write(fs.fs, label.c_str());
                    svm_labels.push_back(it->first);
                } else {
                    return false;
                }

#elif CV_MAJOR_VERSION == 3
                cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();

                svm->setType(svm_type);
                svm->setKernel(kernel_type);
                svm->setDegree(degree);
                svm->setGamma(gamma);
                svm->setCoef0(coef0);
                svm->setC(C);
                svm->setNu(nu);
                svm->setP(p);
                //    svm->setTermCriteria();

                cv::Ptr<cv::ml::TrainData> train_data_struct = cv::ml::TrainData::create(samples,
                                                                                         cv::ml::ROW_SAMPLE,
                                                                                         labels);

                /// train the svm
                std::cout << "Started training for '" << it->first << std::endl;
                if(svm->train(train_data_struct)) {
                    std::cout << "Finished training for '" << it->first << "'!" << std::endl;
                    std::string label = prefix + std::to_string(it->first);
                    fs.writeObj(label, svm);
                    svm_labels.push_back(it->first);
                } else {
                    return false;
                }
#endif
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


                if(gamma_ == 0) {
                    gamma_ = 1.0 / labels.rows;
                }

#if CV_MAJOR_VERSION == 2
                ExtendedSVM svm;

                cv::SVMParams   svm_params_;
                svm_params_.svm_type = svm_type_;
                svm_params_.kernel_type = kernel_type_;
                svm_params_.degree = degree_;
                svm_params_.gamma = gamma_;
                svm_params_.coef0 = coef0_;

                svm_params_.C = C_;
                svm_params_.nu = nu_;
                svm_params_.p = p_;
                //svm_params_.term_crit; // termination criteria

                /// train the svm
                std::cout << "Started training for '" << it->first << std::endl;
                if(svm.train(samples, labels, cv::Mat(), cv::Mat(), svm_params_)) {
                    std::cout << "Finished training for '" << it->first << "'!" << std::endl;
                    std::string label = prefix + std::to_string(it->first);
                    svm.write(fs.fs, label.c_str());
                    svm_labels.push_back(it->first);
                } else {
                    return false;
                }
                /// train the svm
                std::cout << "[SVMEnsemble]: Started training for svm #"
                          << it->first << " with " << samples.rows << std::endl;
                if(svm.train(samples, labels, cv::Mat(), cv::Mat(), svm_params_)) {
                    std::string label = prefix + std::to_string(it->first);
                    svm.write(fs.fs, label.c_str());
                    svm_labels.push_back(it->first);
                    std::cout << "[SVMEnsemble]: Finished training for svm #"
                              << it->first << "!" << std::endl;
                } else {
                    return false;
                }

#elif CV_MAJOR_VERSION == 3
                cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();

                svm->setType(svm_type);
                svm->setKernel(kernel_type);
                svm->setDegree(degree);
                svm->setGamma(gamma);
                svm->setCoef0(coef0);
                svm->setC(C);
                svm->setNu(nu);
                svm->setP(p);
                //    svm->setTermCriteria();


                /// train the svm
                std::cout << "Started training for '" << it->first << std::endl;
                cv::Ptr<cv::ml::TrainData> train_data_struct =
                        cv::ml::TrainData::create(samples,
                                                  cv::ml::ROW_SAMPLE,
                                                  labels);

                if(svm->train(train_data_struct)) {
                    std::cout << "Finished training for '" << it->first << "'!" << std::endl;
                    std::string label = prefix + std::to_string(it->first);
                    fs.writeObj(label, svm);
                    svm_labels.push_back(it->first);
                } else {
                    return false;
                }
                /// train the svm
                std::cout << "[SVMEnsemble]: Started training for svm #"
                          << it->first << " with " << samples.rows << std::endl;
                if(svm->train(train_data_struct)) {
                    std::string label = prefix + std::to_string(it->first);
                    fs.writeObj(label, svm);
                    svm_labels.push_back(it->first);
                    std::cout << "[SVMEnsemble]: Finished training for svm #"
                              << it->first << "!" << std::endl;
                } else {
                    return false;
                }
#endif
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


            if(gamma_ == 0) {
                gamma_ = 1.0 / labels.rows;
            }

#if CV_MAJOR_VERSION == 2
            ExtendedSVM svm;

            cv::SVMParams   svm_params_;
            svm_params_.svm_type = svm_type_;
            svm_params_.kernel_type = kernel_type_;
            svm_params_.degree = degree_;
            svm_params_.gamma = gamma_;
            svm_params_.coef0 = coef0_;

            svm_params_.C = C_;
            svm_params_.nu = nu_;
            svm_params_.p = p_;
            //svm_params_.term_crit; // termination criteria

            /// train the svm
            std::cout << "[SVMEnsemble]: Started training for svm #"
                      << it->first << " with " << samples.rows << std::endl;
            if(svm.train(samples, labels, cv::Mat(), cv::Mat(), svm_params_)) {
                std::string label = prefix + std::to_string(it->first);
                svm.write(fs.fs, label.c_str());
                svm_labels.push_back(it->first);
                std::cout << "[SVMEnsemble]: Finished training for svm #"
                          << it->first << "!" << std::endl;
            } else {
                return false;
            }

#elif CV_MAJOR_VERSION == 3
            cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();

            svm->setType(svm_type);
            svm->setKernel(kernel_type);
            svm->setDegree(degree);
            svm->setGamma(gamma);
            svm->setCoef0(coef0);
            svm->setC(C);
            svm->setNu(nu);
            svm->setP(p);
            //    svm->setTermCriteria();


            /// train the svm
            std::cout << "[SVMEnsemble]: Started training for svm #"
                      << it->first << " with " << samples.rows << std::endl;
            cv::Ptr<cv::ml::TrainData> train_data_struct =
                    cv::ml::TrainData::create(samples,
                                              cv::ml::ROW_SAMPLE,
                                              labels);

            if(svm->train(train_data_struct)) {
                std::string label = prefix + std::to_string(it->first);
                fs.writeObj(label, svm);
                svm_labels.push_back(it->first);
                std::cout << "[SVMEnsemble]: Finished training for svm #"
                          << it->first << "!" << std::endl;
            } else {
                return false;
            }
#endif
        }
    }
    fs << "labels" << svm_labels;
    fs.release();
    return true;
}
