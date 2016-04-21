/// HEADER
#include <csapex_vision_features/keypoint_message.h>

/// PROJECT
#include <csapex_vision/yaml_io.hpp>
#include <csapex/utility/register_msg.h>
#include <csapex/msg/generic_value_message.hpp>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::KeypointMessage)


using namespace csapex;
using namespace connection_types;


KeypointMessage::KeypointMessage()
    : MessageTemplate<std::vector<cv::KeyPoint>, KeypointMessage> ("/")
{}

bool KeypointMessage::isContainer() const
{
    return true;
}

Token::Ptr KeypointMessage::nestedType() const
{
    return makeEmpty<GenericValueMessage<cv::KeyPoint>>();
}

Token::ConstPtr KeypointMessage::nestedValue(std::size_t i) const
{
    GenericValueMessage<cv::KeyPoint>::Ptr v(new GenericValueMessage<cv::KeyPoint>);
    v->value = value.at(i);
    return v;
}
std::size_t KeypointMessage::nestedValueCount() const
{
    return value.size();
}
void KeypointMessage::addNestedValue(const Token::ConstPtr &msg)
{
    auto v = std::dynamic_pointer_cast<GenericValueMessage<cv::KeyPoint> const> (msg);
    if(v) {
        value.push_back(v->value);
    }
}



/// YAML
namespace YAML {
Node convert<csapex::connection_types::KeypointMessage>::encode(const csapex::connection_types::KeypointMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    node["keypoints"] = rhs.value;
    return node;
}

bool convert<csapex::connection_types::KeypointMessage>::decode(const Node& node, csapex::connection_types::KeypointMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }

    convert<csapex::connection_types::Message>::decode(node, rhs);

    rhs.value = node["keypoints"].as<std::vector<cv::KeyPoint> >();
    return true;
}
}
