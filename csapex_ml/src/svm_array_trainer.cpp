/// HEADER
#include "svm_array_trainer.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>

CSAPEX_REGISTER_CLASS(csapex::SVMArrayTrainer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

template<typename T>
inline std::string toString(const T value)
{
    std::stringstream ss;
    ss << value;
    return ss.str();
}

struct ExtendedSVM : public cv::SVM {
    typedef std::shared_ptr<ExtendedSVM> Ptr;

    CvSVMDecisionFunc* get_decision_function()
    {
        return decision_func;
    }

    void print_decision_func()
    {
        std::cout << "alpha: [";
        for(int i = 0 ; i < decision_func->sv_count - 1; ++i) {
            std::cout << decision_func->alpha[i] << ", ";
        }
        std::cout << decision_func->alpha[decision_func->sv_count - 1]
                  << "]" << std::endl;
        std::cout << "rho: " << decision_func->rho  * -1 << std::endl;
    }

    void export_decision_func(cv::FileStorage &fs)
    {
        fs << "svm_alpha" << "[";
        for(int i = 0 ; i < decision_func->sv_count ; ++i)
            fs << decision_func->alpha[i];
        fs << "]";
        fs << "svm_rho" << -decision_func->rho;
    }

    void set_parameters(const cv::SVMParams &params)
    {
        set_params(params);
    }

};

SVMArrayTrainer::SVMArrayTrainer()
{
}

void SVMArrayTrainer::setupParameters(Parameterizable& parameters)
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
}

void SVMArrayTrainer::processCollection(std::vector<FeaturesMessage> &collection)
{
    /// TODO : Mix classes for negative samples?

    if(collection.empty())
        return;

    std::size_t step = collection.front().value.size();
    std::map<int, std::vector<std::size_t>> indices_by_label;

    for(std::size_t i = 0 ; i < collection.size() ; ++i) {
        const FeaturesMessage &fm = collection[i];
        if(fm.value.size() != step)
            throw std::runtime_error("All descriptors must have the same length!");
        indices_by_label[fm.classification].push_back(i);
    }

    std::map<int, ExtendedSVM::Ptr> svms;
    if(svm_params_.svm_type != cv::SVM::ONE_CLASS) {
        if(indices_by_label.find(NEGATIVE) == indices_by_label.end()) {
            throw std::runtime_error("Need negative examples labelled with -1");
        }
        if(indices_by_label.size() == 1) {
            throw std::runtime_error("Multi class SVMs require multiple classes!");
        }
        std::vector<std::size_t> neg_indices = indices_by_label[NEGATIVE];
        indices_by_label.erase(NEGATIVE);
        /// iterate samples
        for(std::size_t i = 0 ; i < indices_by_label.size() ; ++i) {
            auto it = indices_by_label.begin();
            std::advance(it, i);

            /// allocate svm
            ExtendedSVM::Ptr svm(new ExtendedSVM);
            svms[it->first] = svm;

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
            svm.reset(new ExtendedSVM);
            if(svm->train(samples, labels, cv::Mat(), cv::Mat(), params)) {
                std::cout << "Finished training for '" << it->first << std::endl;
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
            ExtendedSVM::Ptr svm(new ExtendedSVM);
            svms[it->first] = svm;

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
            std::cout << "Started training for '" << it->first << std::endl;
            svm.reset(new ExtendedSVM);
            if(svm->train(samples, labels, cv::Mat(), cv::Mat(), params)) {
                std::cout << "Finished training for '" << it->first << std::endl;
            }
        }

    }
    /// write all svms to disk
    cv::FileStorage fs(path_, cv::FileStorage::WRITE);
    const static std::string prefix = "svm_";
    std::vector<int> labels;
    for(auto &svm_entry : svms) {
        std::string label = prefix + toString(svm_entry.first);
        ExtendedSVM::Ptr svm = svm_entry.second;
        svm->write(fs.fs, label.c_str());
        labels.push_back(svm_entry.first);
    }
    fs << "labels" << labels;
    fs.release();
}
