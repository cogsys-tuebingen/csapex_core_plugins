/// HEADER
#include <csapex_ros/duration_message.h>

/// PROJECT
#include <csapex/utility/register_msg.h>

/// SYSTEM
#include <boost/date_time.hpp>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::RosDurationMessage)

using namespace csapex;
using namespace connection_types;

RosDurationMessage::RosDurationMessage()
    : MessageTemplate<ros::Duration, RosDurationMessage> ("/")
{}


/// YAML
namespace YAML {
Node convert<csapex::connection_types::RosDurationMessage>::encode(const csapex::connection_types::RosDurationMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);

    node["duration"] = rhs.value.toNSec();
    return node;
}

bool convert<csapex::connection_types::RosDurationMessage>::decode(const Node& node, csapex::connection_types::RosDurationMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }

    convert<csapex::connection_types::Message>::decode(node, rhs);

    u_int64_t nano = node["duration"].as<u_int64_t>();
    rhs.value = rhs.value.fromNSec(nano);
    return true;
}
}
