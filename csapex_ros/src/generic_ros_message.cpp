/// HEADER
#include <csapex_ros/generic_ros_message.h>

/// PROJECT
#include <csapex/utility/register_msg.h>

/// SYSTEM
#include <topic_tools/shape_shifter.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::GenericRosMessage)

using namespace csapex;
using namespace connection_types;

GenericRosMessage::GenericRosMessage()
    : MessageTemplate<std::shared_ptr<topic_tools::ShapeShifter const>, GenericRosMessage> ("/")
{}


/// YAML
namespace YAML {
Node convert<csapex::connection_types::GenericRosMessage>::encode(const csapex::connection_types::GenericRosMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    return node;
}

bool convert<csapex::connection_types::GenericRosMessage>::decode(const Node& node, csapex::connection_types::GenericRosMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }

    convert<csapex::connection_types::Message>::decode(node, rhs);

    return true;
}
}
