/// HEADER
#include <csapex_opencv/circle_message.h>

/// COMPONENT
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE_WITH_NAME(csapex::connection_types::CircleMessage, CircleMessage)

YAML::Node YAML::convert<csapex::connection_types::CircleMessage>::encode(const csapex::connection_types::CircleMessage& rhs)
{
  YAML::Node node = convert<csapex::connection_types::Message>::encode(rhs);

  node["center_x"] = rhs.value.center_x;
  node["center_x"] = rhs.value.center_y;
  node["radius"] = rhs.value.radius;
  return node;
}

bool YAML::convert<csapex::connection_types::CircleMessage>::decode(const YAML::Node& node, csapex::connection_types::CircleMessage& rhs)
{
  if(!node.IsMap()) {
    return false;
  }

  convert<csapex::connection_types::Message>::decode(node, rhs);
  auto value = node["value"];
  if(!value.IsDefined() || !value.IsMap()) {
    return false;
  }
  rhs.value.center_x = value["center_x"].as<double>();
  rhs.value.center_y = value["center_x"].as<double>();
  rhs.value.radius = value["radius"].as<double>();
  return true;
}
