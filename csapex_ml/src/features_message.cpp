/// HEADER
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::FeaturesMessage)

using namespace csapex;
using namespace connection_types;

FeaturesMessage::FeaturesMessage(Message::Stamp stamp)
    : Message("FeatureMessage", "/", stamp), classification(0)
{}

ConnectionType::Ptr FeaturesMessage::clone() const
{
    Ptr new_msg(new FeaturesMessage);
    new_msg->value = value;
    new_msg->classification = classification;
    return new_msg;
}

ConnectionType::Ptr FeaturesMessage::toType() const
{
    Ptr new_msg(new FeaturesMessage);
    return new_msg;
}



/// YAML
namespace YAML {
Node convert<csapex::connection_types::FeaturesMessage>::encode(const csapex::connection_types::FeaturesMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    node["features"] = rhs.value;
    node["classification"] = rhs.classification;
    node["confidence"] = rhs.confidence;
    return node;
}

bool convert<csapex::connection_types::FeaturesMessage>::decode(const Node& node, csapex::connection_types::FeaturesMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);
    rhs.value = node["features"].as<std::vector<float> >();
    rhs.classification = node["classification"].as<int>();
    if(node["confidence"].IsDefined())
        rhs.confidence = node["confidence"].as<float>();
    else
        rhs.confidence = 1.f;

    return true;
}
}
