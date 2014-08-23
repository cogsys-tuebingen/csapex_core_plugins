/// HEADER
#include <csapex_vision_features/keypoint_message.h>

#include <csapex_vision/yaml_io.hpp>

using namespace csapex;
using namespace connection_types;


KeypointMessage::KeypointMessage()
    : MessageTemplate<std::vector<cv::KeyPoint>, KeypointMessage> ("/")
{}



/// YAML
namespace YAML {
Node convert<csapex::connection_types::KeypointMessage>::encode(const csapex::connection_types::KeypointMessage& rhs)
{
    Node node;
    node["keypoints"] = rhs.value;
    return node;
}

bool convert<csapex::connection_types::KeypointMessage>::decode(const Node& node, csapex::connection_types::KeypointMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }

    rhs.value = node["keypoints"].as<std::vector<cv::KeyPoint> >();
    return true;
}
}
