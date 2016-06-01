/// HEADER
#include "svm.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>

CSAPEX_REGISTER_CLASS(csapex::SVM, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

SVM::SVM() :
    loaded_(false)
{
}

void SVM::setup(NodeModifier& node_modifier)
{
    in_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("features");
    out_ = node_modifier.addOutput<GenericVectorMessage, FeaturesMessage>("labelled features");
}

void SVM::setupParameters(Parameterizable& parameters)
{
    addParameter(csapex::param::ParameterFactory::declarePath("svm path",
                                                      csapex::param::ParameterDescription("Path to a saved svm."),
                                                      true,
                                                      "",
                                                      "*.yaml *.tar.gz"),
                 std::bind(&SVM::load, this));

}

void SVM::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> input = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_);
    std::shared_ptr< std::vector<FeaturesMessage> > output (new std::vector<FeaturesMessage>);
    output->insert(output->end(), input->begin(), input->end());

    if(!loaded_) {
        throw std::runtime_error("No SVM is loaded!");
    }

    std::size_t size = input->size();
    for(std::size_t i = 0 ; i < size ; ++i) {
        cv::Mat sample(output->at(i).value);
        output->at(i).classification = svm_.predict(sample);
    }

    msg::publish<GenericVectorMessage, FeaturesMessage>(out_, output);
}

void SVM::load()
{
    std::string path = readParameter<std::string>("svm path");
    if(path == "")
        return;

    svm_.load(path.c_str(), "svm");
    loaded_ = true;
}
