/// HEADER
#include "empty_labeled_feature_message.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/msg/message.h>
#include <csapex/factory/message_factory.h>
#include <csapex/model/token.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>

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
    output_vec_ = node_modifier.addOutput<GenericVectorMessage, csapex::connection_types::FeaturesMessage>("static feature (vector)");
}

void EmptyLabeledFeaturesMessage::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("class id", 0, 255, 0, 1));
}


void EmptyLabeledFeaturesMessage::process()
{
}

bool EmptyLabeledFeaturesMessage::canTick()
{
    return true;
}

void EmptyLabeledFeaturesMessage::tick()
{
    FeaturesMessage::Ptr static_feature(new FeaturesMessage);
    static_feature->classification = readParameter<int>("class id");
    msg::publish(output_, static_feature);

    std::shared_ptr<std::vector<FeaturesMessage> > static_features(new std::vector<FeaturesMessage>);
    static_features->push_back(*static_feature);
    msg::publish<GenericVectorMessage, FeaturesMessage>(output_vec_, static_features);
}
