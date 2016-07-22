/// HEADER
#include "svm_array_trainer.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>

CSAPEX_REGISTER_CLASS(csapex::SVMTrainer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

struct ExtendedSVM : cv::SVM {
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

    std::vector<ExtendedSVM> svms;
    if(svm_params_.svm_type != cv::SVM::ONE_CLASS) {
        if(indices_by_label.find(NEGATIVE) == indices_by_label.end()) {
            throw std::runtime_error("Need negative examples labelled with -1");
        }
        if(indices_by_label.size() == 1) {
            throw std::runtime_error("Multi class SVMs require multiple classes!");
        }
        svms.resize(indices_by_label.size() - 1);
        /// iterate samples
        /// mixed classes?
    } else {
        if(indices_by_label.find(NEGATIVE) != indices_by_label.end()) {
            indices_by_label.erase(NEGATIVE);
        }
        svms.resize(indices_by_label.size());
        /// iterate samples

    }

    /// save the svms with write and label

//    /// maybe we can use some combinatoric principles to mix classes

//    std::vector<ExtendedSVM> svms;

//    ExtendedSVM     svm;
//    cv::Mat samples(collection.size(), step, CV_32FC1, cv::Scalar::all(0));
//    cv::Mat labels (collection.size(), 1, CV_32FC1, cv::Scalar::all(0));
//    for(int i = 0 ; i < samples.rows ; ++i) {
//        labels.at<float>(i)    = collection.at(i).classification; //  i % 2
//        for(int j = 0 ; j < samples.cols ; ++j) {
//            samples.at<float>(i,j) = collection.at(i).value.at(j);
//        }
//    }

//    if (svm_params_.gamma == 0) {
//        svm_params_.gamma = 1.0 / labels.rows;
//        getParameter("gamma")->set(svm_params_.gamma);
//    }

//    std::cout << "started training" << std::endl;
//    if(svm.train(samples, labels,cv::Mat(), cv::Mat(), svm_params_)) {
//        std::cout << "finished training" << std::endl;
//        svm.print_decision_func();
//        if(save_for_hog_) {
//            cv::FileStorage fs(path_, cv::FileStorage::WRITE);
//            CvSVMDecisionFunc *df = svm.get_decision_function();
//            std::vector<float> coeffs(svm.get_var_count(), 0.f);
//            for(int i = 0 ; i < df->sv_count ; ++i) {
//                const float *sv = svm.get_support_vector(i);
//                for(int j = 0 ; j < svm.get_var_count() ; ++j) {
//                    coeffs[j] += df->alpha[i] * sv[j] * -1;
//                }
//            }

//            fs << "svm_coeffs" << coeffs;
//            svm.export_decision_func(fs);
//            fs.release();
//        } else {
//            svm.save(path_.c_str(), "svm");
//        }
//    } else {
//        throw std::runtime_error("Training failed!");
//    }
}
