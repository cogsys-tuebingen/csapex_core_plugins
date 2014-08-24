/// HEADER
#include <csapex_vision_features/keypoint_message.h>

/// PROJECT
#include <csapex_vision/yaml_io.hpp>
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::KeypointMessage)


using namespace csapex;
using namespace connection_types;


KeypointMessage::KeypointMessage()
    : MessageTemplate<std::vector<cv::KeyPoint>, KeypointMessage> ("/")
{}



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
