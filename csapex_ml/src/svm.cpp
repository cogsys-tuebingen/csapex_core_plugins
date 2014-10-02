/// HEADER
#include "svm.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>

CSAPEX_REGISTER_CLASS(csapex::SVM, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

SVM::SVM() :
    loaded_(false)
{
}

void SVM::setup()
{
    in_ = modifier_->addInput<GenericVectorMessage, FeaturesMessage>("Features");
    out_ = modifier_->addOutput<GenericVectorMessage, FeaturesMessage>("Labeled Features");
}

void SVM::setupParameters()
{
    addParameter(param::ParameterFactory::declarePath("svm path",
                                                      param::ParameterDescription("Path to a saved svm."),
                                                      true,
                                                      "",
                                                      "*.yaml *.tar.gz"),
                 boost::bind(&SVM::load, this));

}

void SVM::process()
{
    boost::shared_ptr<std::vector<FeaturesMessage> const> in = in_->getMessage<GenericVectorMessage, FeaturesMessage>();
    boost::shared_ptr<std::vector<FeaturesMessage> >      out(new std::vector<FeaturesMessage>(in->size()));
    m_.lock();

    if(!loaded_) {
        throw std::runtime_error("No SVM is loaded!");
    }

    for(unsigned int i = 0 ; i < in->size() ; ++i) {
        out->at(i) = in->at(i);
        cv::Mat sample(in->at(i).value);
        out->at(i).classification = svm_.predict(sample);
    }

    m_.unlock();

    out_->publish<GenericVectorMessage, FeaturesMessage>(out);
}

void SVM::load()
{
    std::string path = readParameter<std::string>("svm path");
    if(path == "")
        return;

    m_.lock();
    svm_.load(path.c_str());
    loaded_ = true;
    m_.unlock();
}
