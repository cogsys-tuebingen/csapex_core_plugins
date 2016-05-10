#include "mlp_cv.hpp"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/signal/slot.h>

CSAPEX_REGISTER_CLASS(csapex::MLPCv, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


MLPCv::MLPCv()
    : loaded_(false)
{
}

void MLPCv::setup(NodeModifier &node_modifier)
{
    input_ = node_modifier.addInput<VectorMessage, FeaturesMessage>("Unclassified feature");
    output_ = node_modifier.addOutput<VectorMessage, FeaturesMessage>("Classified features");

    reload_ = node_modifier.addSlot("Reload", std::bind(&MLPCv::loadMLP, this));
}

void MLPCv::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareFileInputPath("file", "mlp.yaml"),
                            std::bind(&MLPCv::loadMLP, this));
}

void MLPCv::loadMLP()
{
    mlp_.load(readParameter<std::string>("file").c_str());
    loaded_ = true;
}

void MLPCv::process()
{
    VectorMessage::ConstPtr input = msg::getMessage<VectorMessage>(input_);
    VectorMessage::Ptr output = VectorMessage::make<FeaturesMessage>();

    if(loaded_) {
        std::size_t n = input->value.size();
        output->value.resize(n);
        for(std::size_t i = 0; i < n; ++i) {
            output->value.at(i) = classify(std::dynamic_pointer_cast<FeaturesMessage const>(input->value.at(i)));
        }

    } else {
        *output = *input;
        node_modifier_->setWarning("cannot classfiy, no forest loaded");
    }

    msg::publish(output_, output);
}

connection_types::FeaturesMessage::Ptr MLPCv::classify(const connection_types::FeaturesMessage::ConstPtr& input)
{
    FeaturesMessage::Ptr result = std::dynamic_pointer_cast<FeaturesMessage>(input->clone());

    cv::Mat feature(1, input->value.size(), CV_32FC1, const_cast<float*>(input->value.data()));
    cv::Mat response;
    mlp_.predict(feature, response);

    cv::Point max;
    cv::minMaxLoc(response, nullptr, nullptr, nullptr, &max);

    result->classification = max.x;
    return result;
}
