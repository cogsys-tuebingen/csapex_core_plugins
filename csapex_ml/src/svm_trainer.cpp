/// HEADER
#include "svm_trainer.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>

/// SYSTEM
#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::SVMTrainer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

SVMTrainer::SVMTrainer() :
    step_(0)
{
}

void SVMTrainer::setup()
{
    in_        = modifier_->addOptionalInput<FeaturesMessage>("Feature");
    in_vector_ = modifier_->addOptionalInput<GenericVectorMessage, FeaturesMessage>("Features");
}

void SVMTrainer::setupParameters()
{

    addParameter(param::ParameterFactory::declareFileOutputPath("path",
                                                                param::ParameterDescription("File to write svm to."),
                                                                "",
                                                                "*.yaml *.tar.gz"));

    addParameter(param::ParameterFactory::declareTrigger("train",
                                                         param::ParameterDescription("Train using obtained data!")),
                 boost::bind(&SVMTrainer::train, this));

    addParameter(param::ParameterFactory::declareTrigger("clear",
                                                         param::ParameterDescription("Clear buffered data!")),
                 boost::bind(&SVMTrainer::clear, this));


    std::map<std::string, int> kernel_types =
            boost::assign::map_list_of
            ("LINEAR", cv::SVM::LINEAR)
            ("POLY", cv::SVM::POLY)
            ("RBF", cv::SVM::RBF)
            ("SIGMOID", cv::SVM::SIGMOID);

    param::Parameter::Ptr kernel_param =
            param::ParameterFactory::declareParameterSet("kernel type",
                                                         param::ParameterDescription("Kernel type to be trained."),
                                                         kernel_types,
                                                         (int) cv::SVM::RBF);
    addParameter(kernel_param);

    std::map<std::string, int> svm_types =
            boost::assign::map_list_of
            ("C_SVC", cv::SVM::C_SVC)
            ("NU_SVC", cv::SVM::NU_SVC)
            ("ONE_CLASS", cv::SVM::ONE_CLASS)
            ("EPS_SVR", cv::SVM::EPS_SVR)
            ("NU_SVR", cv::SVM::NU_SVR);

    param::Parameter::Ptr svm_param =
            param::ParameterFactory::declareParameterSet("svm type",
                                                         param::ParameterDescription("SVM type to be trained."),
                                                         svm_types,
                                                         (int) cv::SVM::C_SVC);

    addParameter(svm_param);

    std::function<bool()> deg_cond    = (boost::bind(&param::Parameter::as<int>, kernel_param.get()) == cv::SVM::POLY);
    std::function<bool()> gamma_cond  = (boost::bind(&param::Parameter::as<int>, kernel_param.get()) == cv::SVM::POLY  ||
                                           boost::bind(&param::Parameter::as<int>, kernel_param.get()) == cv::SVM::RBF   ||
                                           boost::bind(&param::Parameter::as<int>, kernel_param.get()) == cv::SVM::SIGMOID);
    std::function<bool()> coeff0_cond = (boost::bind(&param::Parameter::as<int>, kernel_param.get()) == cv::SVM::POLY  ||
                                           boost::bind(&param::Parameter::as<int>, kernel_param.get()) == cv::SVM::SIGMOID);
    std::function<bool()> cvalue_cond = (boost::bind(&param::Parameter::as<int>, svm_param.get()) == cv::SVM::C_SVC    ||
                                           boost::bind(&param::Parameter::as<int>, svm_param.get()) == cv::SVM::EPS_SVR  ||
                                           boost::bind(&param::Parameter::as<int>, svm_param.get()) == cv::SVM::NU_SVR);
    std::function<bool()> nu_cond     = (boost::bind(&param::Parameter::as<int>, svm_param.get()) == cv::SVM::NU_SVC    ||
                                           boost::bind(&param::Parameter::as<int>, svm_param.get()) == cv::SVM::ONE_CLASS ||
                                           boost::bind(&param::Parameter::as<int>, svm_param.get()) == cv::SVM::NU_SVR);
    std::function<bool()> p_cond      = (boost::bind(&param::Parameter::as<int>, svm_param.get()) == cv::SVM::EPS_SVR);


    addConditionalParameter(param::ParameterFactory::declareRange<double>("degree", 0.0, M_PI * 2, 0.0, 0.05),
                            deg_cond);
    addConditionalParameter(param::ParameterFactory::declareRange<double>("gamma", -M_PI, M_PI, 1.0, 0.05),
                            gamma_cond);
    addConditionalParameter(param::ParameterFactory::declareRange<double>("coef0", -M_PI, M_PI, 0.0, 0.05),
                            coeff0_cond);
    addConditionalParameter(param::ParameterFactory::declareRange<double>("C", -M_PI, M_PI, 1.0, 0.05),
                            cvalue_cond);
    addConditionalParameter(param::ParameterFactory::declareRange<double>("nu", 0.0, 1.0, 0.0, 0.05),
                            nu_cond);
    addConditionalParameter(param::ParameterFactory::declareRange<double>("p", -M_PI, M_PI, 0.0, 0.05),
                            p_cond);
}

void SVMTrainer::process()
{
    if(in_->hasMessage()) {
        FeaturesMessage::ConstPtr msg = in_->getMessage<FeaturesMessage>();
        m_.lock();

        if(step_ == 0)
            step_ = msg->value.size();
        if(step_ != msg->value.size())
            throw std::runtime_error("Inconsistent feature length!");

        msgs_.push_back(*msg);
        m_.unlock();
    }

    if(in_vector_->hasMessage()) {
        std::shared_ptr<std::vector<FeaturesMessage> const> in =
                in_vector_->getMessage<GenericVectorMessage, FeaturesMessage>();

        m_.lock();
        for(unsigned int i = 0 ; i < in->size() ; ++i) {
            if(step_ == 0)
                step_ = in->at(i).value.size();
            if(step_ != in->at(i).value.size())
                throw std::runtime_error("Inconsistent feature length!");
        }

        msgs_.insert(msgs_.end(), in->begin(), in->end());
        m_.unlock();
    }
}

void SVMTrainer::train()
{
    m_.lock();
    std::vector<FeaturesMessage> data = msgs_;
    m_.unlock();

    cv::SVM         svm;
    cv::SVMParams   parameters;
    parameters.kernel_type = readParameter<int>("kernel type");
    parameters.svm_type    = readParameter<int>("svm type");
    parameters.degree      = readParameter<double>("degree");
    parameters.gamma       = readParameter<double>("gamma");
    parameters.coef0       = readParameter<double>("coef0");
    parameters.C           = readParameter<double>("C");
    parameters.nu          = readParameter<double>("nu");
    parameters.p           = readParameter<double>("p");

    cv::Mat samples(data.size(), step_, CV_32FC1, cv::Scalar::all(0));
    cv::Mat labels (data.size(), 1, CV_32FC1, cv::Scalar::all(0));
    for(int i = 0 ; i < samples.rows ; ++i) {
        for(int j = 0 ; j < samples.cols ; ++j) {
            samples.at<float>(i,j) = data.at(i).value.at(j);
            labels.at<float>(i)    = data.at(i).classification;
        }
    }

    if(svm.train(samples, labels, cv::Mat(), cv::Mat(), parameters)) {
        std::string path = readParameter<std::string>("path");
        svm.save(path.c_str());
    } else {
        throw std::runtime_error("Training failed!");
    }

}

void SVMTrainer::clear()
{
    m_.lock();
    msgs_.clear();
    step_ = 0;
    m_.unlock();
}
