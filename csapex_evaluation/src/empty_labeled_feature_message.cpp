/// HEADER
#include "empty_labeled_feature_message.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/msg/message.h>
#include <csapex/msg/message_factory.h>
#include <csapex/model/connection_type.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>

/// CONCRETE MESSAGES
#include <csapex_ml/features_message.h>

CSAPEX_REGISTER_CLASS(csapex::EmptyLabeledFeaturesMessage, csapex::Node)

using namespace csapex;
using namespace connection_types;

EmptyLabeledFeaturesMessage::EmptyLabeledFeaturesMessage()
{
}

void EmptyLabeledFeaturesMessage::setup(NodeModifier& node_modifier)
{
    output_ = node_modifier.addOutput<FeaturesMessage>("static feature");
}

void EmptyLabeledFeaturesMessage::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareRange("class id", 0, 255, 0, 1));
}


void EmptyLabeledFeaturesMessage::process()
{
}

void EmptyLabeledFeaturesMessage::tick()
{
    FeaturesMessage::Ptr static_feature(new FeaturesMessage);
    static_feature->classification = readParameter<int>("class id");
    msg::publish(output_, static_feature);
}
