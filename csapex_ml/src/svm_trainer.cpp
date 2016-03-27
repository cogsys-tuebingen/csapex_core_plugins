/// HEADER
#include "svm_trainer.h"

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
        std::cout << "rho: " << decision_func->rho << std::endl;
    }

};

SVMTrainer::SVMTrainer() :
    step_(0)
{
}

void SVMTrainer::setup(NodeModifier& node_modifier)
{
    in_vector_ = node_modifier.addOptionalInput<VectorMessage, FeaturesMessage>("Features");
}

void SVMTrainer::setupParameters(Parameterizable& parameters)
{

    addParameter(csapex::param::ParameterFactory::declareFileOutputPath("path",
                                                                        csapex::param::ParameterDescription("File to write svm to."),
                                                                        "",
                                                                        "*.yaml *.tar.gz"),
                 path_);

    addParameter(param::ParameterFactory::declareBool("save for HOG",
                                                      param::ParameterDescription("Save precomputed vector for HOG."),
                                                      false),
                 save_for_hog_);

    addParameter(csapex::param::ParameterFactory::declareTrigger("train",
                                                                 csapex::param::ParameterDescription("Train using obtained data!")),
                 std::bind(&SVMTrainer::train, this));

    addParameter(csapex::param::ParameterFactory::declareTrigger("clear",
                                                                 csapex::param::ParameterDescription("Clear buffered data!")),
                 std::bind(&SVMTrainer::clear, this));


    std::map<std::string, int> kernel_types = {
        {"LINEAR", cv::SVM::LINEAR},
        {"POLY", cv::SVM::POLY},
        {"RBF", cv::SVM::RBF},
        {"SIGMOID", cv::SVM::SIGMOID}
    };

    csapex::param::Parameter::Ptr kernel_param =
            csapex::param::ParameterFactory::declareParameterSet("kernel type",
                                                                 csapex::param::ParameterDescription("Kernel type to be trained."),
                                                                 kernel_types,
                                                                 (int) cv::SVM::RBF);
    parameters.addParameter(kernel_param);

    std::map<std::string, int> svm_types = {
        {"C_SVC", cv::SVM::C_SVC},
        {"NU_SVC", cv::SVM::NU_SVC},
        {"ONE_CLASS", cv::SVM::ONE_CLASS},
        {"EPS_SVR", cv::SVM::EPS_SVR},
        {"NU_SVR", cv::SVM::NU_SVR}
    };

    csapex::param::Parameter::Ptr svm_param =
            csapex::param::ParameterFactory::declareParameterSet("svm type",
                                                                 csapex::param::ParameterDescription("SVM type to be trained."),
                                                                 svm_types,
                                                                 (int) cv::SVM::C_SVC);

    parameters.addParameter(svm_param);

    std::function<bool()> deg_cond    = [kernel_param]() -> bool {
            auto t = kernel_param->as<int>();
            return t == cv::SVM::POLY;
};
    std::function<bool()> gamma_cond  = [kernel_param]() -> bool {
            auto t = kernel_param->as<int>();
            return t == cv::SVM::POLY || t == cv::SVM::RBF || cv::SVM::SIGMOID;
};

    std::function<bool()> coeff0_cond = [kernel_param]() -> bool {
            auto t = kernel_param->as<int>();
            return t == cv::SVM::POLY || t == cv::SVM::SIGMOID;
};
    std::function<bool()> cvalue_cond = [kernel_param]() -> bool {
            auto t = kernel_param->as<int>();
            return t == cv::SVM::C_SVC || t == cv::SVM::EPS_SVR || cv::SVM::NU_SVR;
};
    std::function<bool()> nu_cond     = [kernel_param]() -> bool {
            auto t = kernel_param->as<int>();
            return t == cv::SVM::ONE_CLASS || t == cv::SVM::NU_SVR || cv::SVM::EPS_SVR;
};
    std::function<bool()> p_cond      = [kernel_param]() -> bool {
            auto t = kernel_param->as<int>();
            return t == cv::SVM::EPS_SVR;
};


    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareRange<double>("degree", 0.0, M_PI * 2, 0.0, 0.05),
                                       deg_cond);
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareRange<double>("gamma", -M_PI, M_PI, 1.0, 0.05),
                                       gamma_cond);
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareRange<double>("coef0", -M_PI, M_PI, 0.0, 0.05),
                                       coeff0_cond);
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareRange<double>("C", -M_PI, M_PI, 1.0, 0.05),
                                       cvalue_cond);
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareRange<double>("nu", 0.0, 1.0, 0.0, 0.05),
                                       nu_cond);
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareRange<double>("p", -M_PI, M_PI, 0.0, 0.05),
                                       p_cond);
}

void SVMTrainer::process()
{
    VectorMessage::ConstPtr in =
            msg::getMessage<VectorMessage>(in_vector_);

    for(auto entry : in->value) {
        FeaturesMessage::ConstPtr feature = std::dynamic_pointer_cast<FeaturesMessage const>(entry);

        if(step_ == 0)
            step_ = feature->value.size();
        if(step_ != feature->value.size())
            throw std::runtime_error("Inconsistent feature length!");

        msgs_.push_back(feature);
    }
}

void SVMTrainer::train()
{
    ExtendedSVM     svm;
    cv::SVMParams   parameters;
    parameters.kernel_type = readParameter<int>("kernel type");
    parameters.svm_type    = readParameter<int>("svm type");
    parameters.degree      = readParameter<double>("degree");
    parameters.gamma       = readParameter<double>("gamma");
    parameters.coef0       = readParameter<double>("coef0");
    parameters.C           = readParameter<double>("C");
    parameters.nu          = readParameter<double>("nu");
    parameters.p           = readParameter<double>("p");

    cv::Mat samples(msgs_.size(), step_, CV_32FC1, cv::Scalar::all(0));
    cv::Mat labels (msgs_.size(), 1, CV_32FC1, cv::Scalar::all(0));
    for(int i = 0 ; i < samples.rows ; ++i) {
        for(int j = 0 ; j < samples.cols ; ++j) {
            samples.at<float>(i,j) = msgs_.at(i)->value.at(j);
            labels.at<float>(i)    = msgs_.at(i)->classification; //  i % 2
        }
    }

    std::cout << "started training" << std::endl;
    if(svm.train(samples, labels, cv::Mat(), cv::Mat(), parameters)) {
        std::cout << "finished trainding" << std::endl;
        if(save_for_hog_) {
            cv::FileStorage fs(path_, cv::FileStorage::WRITE);
            CvSVMDecisionFunc *df = svm.get_decision_function();
            std::vector<float> coeffs(svm.get_var_count(), 0.f);
            for(int i = 0 ; i < df->sv_count ; ++i) {
                const float *sv = svm.get_support_vector(i);
                for(int j = 0 ; j < svm.get_var_count() ; ++j) {
                    coeffs[j] += df->alpha[i] * sv[j];
                }
            }
            fs << "coeffs" << coeffs;
            fs << "rho" << df->rho;
            fs.release();
        } else {
            svm.save(path_.c_str(), "svm");
        }
    } else {
        throw std::runtime_error("Training failed!");
    }

}

void SVMTrainer::clear()
{
    msgs_.clear();
    step_ = 0;
}
