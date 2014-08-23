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




/// YAML
namespace YAML {
Node convert<csapex::connection_types::FeaturesMessage>::encode(const csapex::connection_types::FeaturesMessage& rhs)
{
    Node node;
    node["features"] = rhs.value;
    node["classification"] = rhs.classification;
    return node;
}

bool convert<csapex::connection_types::FeaturesMessage>::decode(const Node& node, csapex::connection_types::FeaturesMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }

    rhs.value = node["features"].as<std::vector<float> >();
    rhs.classification = node["classification"].as<int>();
    return true;
}
}
