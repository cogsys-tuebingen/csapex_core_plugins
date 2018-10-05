/// HEADER
#include "svm_trainer.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

#include "extended_svm.hpp"

CSAPEX_REGISTER_CLASS(csapex::SVMTrainer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

SVMTrainer::SVMTrainer()
{
}

void SVMTrainer::setup(NodeModifier& modifier)
{
    CollectionNode<connection_types::FeaturesMessage>::setup(modifier);
}

void SVMTrainer::setupParameters(Parameterizable& parameters)
{
    CollectionNode<connection_types::FeaturesMessage>::setupParameters(parameters);

    parameters.addParameter(param::factory::declareFileOutputPath("svm/path", param::ParameterDescription("File to write svm to."), "", "*.yaml *.tar.gz"), path_);

    parameters.addParameter(param::factory::declareBool("svm/save_coeffs", param::ParameterDescription("Save precomputed vector for HOG."), false), save_for_hog_);

#if CV_MAJOR_VERSION == 2
    std::map<std::string, int> kernel_types = { { "LINEAR", cv::SVM::LINEAR }, { "POLY", cv::SVM::POLY }, { "RBF", cv::SVM::RBF }, { "SIGMOID", cv::SVM::SIGMOID } };
#elif CV_MAJOR_VERSION == 3
    std::map<std::string, int> kernel_types = {
        { "LINEAR", cv::ml::SVM::LINEAR }, { "POLY", cv::ml::SVM::POLY }, { "RBF", cv::ml::SVM::RBF }, { "SIGMOID", cv::ml::SVM::SIGMOID }  //,
                                                                                                                                            //{"CHI2", cv::ml::SVM::CHI2},
                                                                                                                                            //{"INTER", cv::ml::SVM::INTER}
    };

#endif

#if CV_MAJOR_VERSION == 2
    typedef cv::SVM SVM;
#elif CV_MAJOR_VERSION == 3
    typedef cv::ml::SVM SVM;
#endif

    parameters.addParameter(param::factory::declareParameterSet("svm/kernel_type", csapex::param::ParameterDescription("Kernel type to be trained."), kernel_types, (int)SVM::RBF), kernel_type_);

    std::map<std::string, int> svm_types = { { "C_SVC", SVM::C_SVC }, { "NU_SVC", SVM::NU_SVC }, { "ONE_CLASS", SVM::ONE_CLASS }, { "EPS_SVR", SVM::EPS_SVR }, { "NU_SVR", SVM::NU_SVR } };

    parameters.addParameter(param::factory::declareParameterSet("svm type", csapex::param::ParameterDescription("SVM type to be trained."), svm_types, (int)SVM::EPS_SVR), svm_type_);

    std::function<bool()> deg_cond = [this]() { return kernel_type_ == SVM::POLY; };

    std::function<bool()> gamma_cond = [this]() { return kernel_type_ == SVM::POLY || kernel_type_ == SVM::RBF || kernel_type_ == SVM::SIGMOID; };

    std::function<bool()> coeff0_cond = [this]() { return kernel_type_ == SVM::POLY || kernel_type_ == SVM::SIGMOID; };

    std::function<bool()> c_cond = [this]() { return svm_type_ == SVM::C_SVC || svm_type_ == SVM::EPS_SVR || svm_type_ == SVM::NU_SVR; };

    std::function<bool()> nu_cond = [this]() { return svm_type_ == SVM::ONE_CLASS || svm_type_ == SVM::NU_SVR || svm_type_ == SVM::EPS_SVR; };

    std::function<bool()> p_cond = [this]() { return svm_type_ == SVM::EPS_SVR; };

    parameters.addConditionalParameter(param::factory::declareRange<double>("degree", 0.0, 9.0, 3.0, 1.0), deg_cond, degree_);
    parameters.addConditionalParameter(param::factory::declareRange<double>("gamma", 0.0, 10.0, 0.0, 0.01), gamma_cond, gamma_);
    parameters.addConditionalParameter(param::factory::declareRange<double>("coef0", -10.0, 10.0, 0.0, 0.01), coeff0_cond, coef0_);
    parameters.addConditionalParameter(param::factory::declareRange<double>("C", 0.0, 10.0, 0.01, 0.01), c_cond, C_);
    parameters.addConditionalParameter(param::factory::declareRange<double>("nu", 0.0, 1.0, 0.5, 0.01), nu_cond, nu_);
    parameters.addConditionalParameter(csapex::param::factory::declareRange<double>("p", 0.0, 1.0, 0.1, 0.01), p_cond, p_);
}

bool SVMTrainer::processCollection(std::vector<FeaturesMessage>& collection)
{
    std::size_t step = collection.front().value.size();
    for (const FeaturesMessage& fm : collection) {
        if (fm.value.size() != step)
            throw std::runtime_error("All descriptors must have the same length!");
    }

    cv::Mat samples(collection.size(), step, CV_32FC1, cv::Scalar::all(0));
    cv::Mat labels(collection.size(), 1, CV_32FC1, cv::Scalar::all(0));
    for (int i = 0; i < samples.rows; ++i) {
        labels.at<float>(i) = collection.at(i).classification;  //  i % 2
        for (int j = 0; j < samples.cols; ++j) {
            samples.at<float>(i, j) = collection.at(i).value.at(j);
        }
    }

    if (gamma_ == 0) {
        gamma_ = 1.0 / labels.rows;
        getParameter("gamma")->set(gamma_);
    }

    std::cout << "[SVM]: started training with " << samples.rows << " samples!" << std::endl;

#if CV_MAJOR_VERSION == 2
    ExtendedSVM svm;

    cv::SVMParams svm_params_;
    svm_params_.svm_type = svm_type_;
    svm_params_.kernel_type = kernel_type_;
    svm_params_.degree = degree_;
    svm_params_.gamma = gamma_;
    svm_params_.coef0 = coef0_;

    svm_params_.C = C_;
    svm_params_.nu = nu_;
    svm_params_.p = p_;
    // svm_params_.term_crit; // termination criteria
    if (svm.train(samples, labels, cv::Mat(), cv::Mat(), svm_params_)) {
        std::cout << "[SVM]: Finished training!" << std::endl;
        if (save_for_hog_) {
            cv::FileStorage fs(path_, cv::FileStorage::WRITE);
            CvSVMDecisionFunc* df = svm.get_decision_function();
            std::vector<float> coeffs(svm.get_var_count(), 0.f);
            for (int i = 0; i < df->sv_count; ++i) {
                const float* sv = svm.get_support_vector(i);
                for (int j = 0; j < svm.get_var_count(); ++j) {
                    coeffs[j] += df->alpha[i] * sv[j] * -1;
                }
            }

            fs << "svm_coeffs" << coeffs;
            svm.export_decision_func(fs);
            fs.release();
        } else {
            svm.save(path_.c_str(), "svm");
        }
    } else {
        return false;
    }

#elif CV_MAJOR_VERSION == 3
    cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();

    svm->setType(svm_type_);
    svm->setKernel(kernel_type_);
    svm->setDegree(degree_);
    svm->setGamma(gamma_);
    svm->setCoef0(coef0_);
    svm->setC(C_);
    svm->setNu(nu_);
    svm->setP(p_);
    //    svm->setTermCriteria();

    labels.convertTo(labels, CV_32SC1);
    cv::Ptr<cv::ml::TrainData> train_data_struct = cv::ml::TrainData::create(samples, cv::ml::ROW_SAMPLE, labels);

    if (svm->train(train_data_struct)) {
        std::cout << "[SVM]: Finished training!" << std::endl;
        if (save_for_hog_) {
            cv::FileStorage fs(path_, cv::FileStorage::WRITE);
            cv::Mat supvec = svm->getSupportVectors();
            //            CvSVMDecisionFunc *df = svm.get_decision_function();
            std::vector<float> alpha;
            std::vector<int> idx;
            auto rho = svm->getDecisionFunction(0, alpha, idx);

            std::vector<float> coeffs(svm->getVarCount(), 0.f);
            for (int i = 0; i < (int)alpha.size(); ++i) {
                std::vector<float> sv = supvec.row(idx.at(i));
                for (int j = 0; j < svm->getVarCount(); ++j) {
                    coeffs[j] += alpha[i] * sv[j] * -1;
                }
            }

            fs << "svm_coeffs" << coeffs;
            fs << "svm_alpha"
               << "[";
            for (int i = 0; i < (int)alpha.size(); ++i)
                fs << alpha.at(i);
            fs << "]";
            fs << "svm_rho" << rho;
            fs.release();
        } else {
            svm->save(path_.c_str());
        }
    } else {
        return false;
    }
#endif

    return true;
}
