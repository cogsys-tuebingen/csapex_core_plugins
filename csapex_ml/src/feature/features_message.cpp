/// HEADER
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::FeaturesMessage)

using namespace csapex;
using namespace connection_types;

FeaturesMessage::FeaturesMessage(Type type, Message::Stamp stamp) : Message("FeaturesMessage", "/", stamp), type(type), classification(0)
{
}
FeaturesMessage::FeaturesMessage(Message::Stamp stamp) : FeaturesMessage(Type::CLASSIFICATION, stamp)
{
}

/// YAML
namespace YAML
{
Node convert<csapex::connection_types::FeaturesMessage>::encode(const csapex::connection_types::FeaturesMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    node["features"] = rhs.value;
    node["type"] = (int)rhs.type;
    switch (rhs.type) {
        case FeaturesMessage::Type::CLASSIFICATION:
            node["classification"] = rhs.classification;
            break;
        case FeaturesMessage::Type::REGRESSION:
            node["regression_result"] = rhs.regression_result;
            break;
        default:
            break;
    }
    node["confidence"] = rhs.confidence;
    return node;
}

bool convert<csapex::connection_types::FeaturesMessage>::decode(const Node& node, csapex::connection_types::FeaturesMessage& rhs)
{
    if (!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);
    rhs.value = node["features"].as<std::vector<float>>();

    // For backward compatibility
    rhs.type = FeaturesMessage::Type::CLASSIFICATION;
    if (node["type"].IsDefined()) {
        rhs.type = (FeaturesMessage::Type)node["type"].as<int>();
    }
    switch (rhs.type) {
        case FeaturesMessage::Type::CLASSIFICATION:
            rhs.classification = node["classification"].as<int>();
            break;
        case FeaturesMessage::Type::REGRESSION:
            rhs.regression_result = node["regression_result"].as<std::vector<float>>();
            break;
        default:
            throw std::logic_error("Unkown FeatureMessage type.");
    }

    if (node["confidence"].IsDefined())
        rhs.confidence = node["confidence"].as<float>();
    else
        rhs.confidence = 1.f;

    return true;
}
}  // namespace YAML
