/// HEADER
#include <csapex_vision_features/match_message.h>

/// PROJECT
#include <csapex_vision/yaml_io.hpp>
#include <csapex/utility/register_msg.h>
#include <csapex/msg/generic_value_message.hpp>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::MatchMessage)


using namespace csapex;
using namespace connection_types;


MatchMessage::MatchMessage()
    : MessageTemplate<std::vector<cv::DMatch>, MatchMessage> ("/")
{}

bool MatchMessage::isContainer() const
{
    return true;
}

ConnectionType::Ptr MatchMessage::nestedType() const
{
    return makeEmpty<GenericValueMessage<cv::DMatch>>();
}

ConnectionType::ConstPtr MatchMessage::nestedValue(std::size_t i) const
{
    GenericValueMessage<cv::DMatch>::Ptr v(new GenericValueMessage<cv::DMatch>);
    v->value = value.at(i);
    return v;
}
std::size_t MatchMessage::nestedValueCount() const
{
    return value.size();
}
void MatchMessage::addNestedValue(const ConnectionType::ConstPtr &msg)
{
    auto v = std::dynamic_pointer_cast<GenericValueMessage<cv::DMatch> const> (msg);
    if(v) {
        value.push_back(v->value);
    }
}



/// YAML
namespace YAML {
Node convert<csapex::connection_types::MatchMessage>::encode(const csapex::connection_types::MatchMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    node["matches"] = rhs.value;
    return node;
}

bool convert<csapex::connection_types::MatchMessage>::decode(const Node& node, csapex::connection_types::MatchMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }

    convert<csapex::connection_types::Message>::decode(node, rhs);

    rhs.value = node["matches"].as<std::vector<cv::DMatch> >();
    return true;
}
}
