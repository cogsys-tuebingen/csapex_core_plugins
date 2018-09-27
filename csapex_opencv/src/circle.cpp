
/// HEADER
#include <csapex_opencv/circle.h>

/// COMPONENT
#include <yaml-cpp/yaml.h>
#include <csapex/utility/register_msg.h>

using namespace csapex;

CSAPEX_REGISTER_MESSAGE_WITH_NAME(csapex::connection_types::CircleMessage, CircleMessage)

SerializationBuffer& csapex::operator << (SerializationBuffer& data, const Circle& c)
{
  data << c.center_x;
  data << c.center_y;
  data << c.radius;
  return data;
}

const SerializationBuffer& csapex::operator >> (const SerializationBuffer& data, Circle& c)
{
  data >> c.center_x;
  data >> c.center_y;
  data >> c.radius;
  return data;
}

/// YAML
namespace YAML {
Node convert<csapex::connection_types::CircleMessage>::encode(const csapex::connection_types::CircleMessage& rhs)
{
  Node node = convert<csapex::connection_types::Message>::encode(rhs);

  node["center_x"] = rhs.value.center_x;
  node["center_x"] = rhs.value.center_y;
  node["radius"] = rhs.value.radius;
  return node;
}

bool convert<csapex::connection_types::CircleMessage>::decode(const Node& node, csapex::connection_types::CircleMessage& rhs)
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
}
