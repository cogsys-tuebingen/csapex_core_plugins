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
    in_ = node_modifier.addInput<VectorMessage, FeaturesMessage>("features");
    out_ = node_modifier.addOutput<VectorMessage, FeaturesMessage>("labelled features");
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
    VectorMessage::ConstPtr features_msg = msg::getMessage<VectorMessage>(in_);

    VectorMessage::Ptr out_msg(VectorMessage::make<FeaturesMessage>());

    if(!loaded_) {
        throw std::runtime_error("No SVM is loaded!");
    }

    for(ConnectionType::ConstPtr fm : features_msg->value) {
        FeaturesMessage::ConstPtr msg = std::dynamic_pointer_cast<FeaturesMessage const>(fm);
        FeaturesMessage::Ptr out(new FeaturesMessage());
        out->value = msg->value;
        cv::Mat sample(msg->value);
        out->classification = svm_.predict(sample);
        out_msg->value.push_back(out);
    }

    msg::publish(out_, out_msg);
}

void SVM::load()
{
    std::string path = readParameter<std::string>("svm path");
    if(path == "")
        return;

    svm_.load(path.c_str(), "svm");
    loaded_ = true;
}
