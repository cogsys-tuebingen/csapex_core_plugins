/// HEADER
#include "confidence_class_toggle.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/signal/slot.h>
#include <csapex_ml/features_message.h>

CSAPEX_REGISTER_CLASS(csapex::ConfidenceClassToggle, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

ConfidenceClassToggle::ConfidenceClassToggle()
{
}

void ConfidenceClassToggle::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareValue("class from", 0));
    parameters.addParameter(csapex::param::ParameterFactory::declareValue("class to", 1));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("threshold", 0.0, 1.0, 0.9, 0.01));
}

void ConfidenceClassToggle::setup(NodeModifier& node_modifier)
{
    in_  = node_modifier.addInput<GenericVectorMessage, csapex::connection_types::FeaturesMessage>("Features");
    out_ = node_modifier.addOutput<GenericVectorMessage, csapex::connection_types::FeaturesMessage>("Toggled");
}

void ConfidenceClassToggle::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> in = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_);
    std::shared_ptr<std::vector<FeaturesMessage>>       out(new std::vector<connection_types::FeaturesMessage>(in->size()));

    int from = readParameter<int>("class from");
    int to   = readParameter<int>("class to");
    float thresh = readParameter<double>("threshold");

    auto it_in = in->begin();
    auto it_out = out->begin();
    for(; it_in != in->end() ; ++it_in, ++it_out) {
        *it_out = *it_in;
        if(it_out->classification == from &&
                it_out->confidence < thresh) {
            it_out->classification = to;
            it_out->confidence = 1 - it_out->confidence;
        }
    }

    msg::publish<GenericVectorMessage, FeaturesMessage>(out_, out);
}
