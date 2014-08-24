/// HEADER
#include <csapex_transform/time_stamp_message.h>

/// PROJECT
#include <csapex/utility/register_msg.h>

/// SYSTEM
#include <boost/date_time.hpp>


CSAPEX_REGISTER_MESSAGE(csapex::connection_types::TimeStampMessage)

using namespace csapex;
using namespace connection_types;

TimeStampMessage::TimeStampMessage()
    : MessageTemplate<ros::Time, TimeStampMessage> ("/")
{}


/// YAML
namespace YAML {
Node convert<csapex::connection_types::TimeStampMessage>::encode(const csapex::connection_types::TimeStampMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);

    node["time"] = boost::posix_time::to_simple_string(rhs.value.toBoost());
    return node;
}

bool convert<csapex::connection_types::TimeStampMessage>::decode(const Node& node, csapex::connection_types::TimeStampMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }

    convert<csapex::connection_types::Message>::decode(node, rhs);

    std::string str = node["time"].as<std::string>();
    rhs.value = ros::Time::fromBoost(boost::posix_time::time_from_string(str));
    return true;
}
}
