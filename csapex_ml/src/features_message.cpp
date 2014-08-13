/// HEADER
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex/utility/assert.h>

using namespace csapex;
using namespace connection_types;

FeaturesMessage::FeaturesMessage()
    : Message("FeatureMessage", "/"), classification(0)
{}

ConnectionType::Ptr FeaturesMessage::clone() {
    Ptr new_msg(new FeaturesMessage);
    new_msg->value = value;
    new_msg->classification = classification;
    return new_msg;
}

ConnectionType::Ptr FeaturesMessage::toType() {
    Ptr new_msg(new FeaturesMessage);
    return new_msg;
}

ConnectionType::Ptr FeaturesMessage::make(){
    Ptr new_msg(new FeaturesMessage);
    return new_msg;
}


void FeaturesMessage::writeYaml(YAML::Emitter &yaml) const
{
    yaml << YAML::Flow << YAML::Key << "features" << YAML::Value;
    yaml << YAML::BeginSeq;
    std::vector<float>::const_iterator r = value.begin();
    for(; r != value.end(); ++r) {
        yaml << *r;
    }
    yaml << YAML::EndSeq;
    yaml << YAML::Key << "classification" << YAML::Value << classification;
}

void FeaturesMessage::readYaml(const YAML::Node &node)
{
    apex_assert_hard(node.Type() == YAML::NodeType::Sequence);
}
