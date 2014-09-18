#include <csapex_core_plugins/point_message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::PointMessage)

using namespace csapex;
using namespace connection_types;

PointMessage::PointMessage()
    : Message(type<PointMessage>::name(), "/")
{}


ConnectionType::Ptr PointMessage::make()
{
    Ptr new_msg(new PointMessage);
    return new_msg;
}

ConnectionType::Ptr PointMessage::clone()
{
    Ptr new_msg(new PointMessage);
    new_msg->x = x;
    new_msg->y = y;
    return new_msg;
}

ConnectionType::Ptr PointMessage::toType()
{
    return make();
}


/// YAML
namespace YAML {
Node convert<csapex::connection_types::PointMessage>::encode(const csapex::connection_types::PointMessage& rhs)
{
    Node node;
    node["x"] = rhs.x;
    node["y"] = rhs.y;
    return node;
}

bool convert<csapex::connection_types::PointMessage>::decode(const Node& node, csapex::connection_types::PointMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }
    rhs.x = node["x"].as<float>();
    rhs.y = node["y"].as<float>();
    return true;
}
}

