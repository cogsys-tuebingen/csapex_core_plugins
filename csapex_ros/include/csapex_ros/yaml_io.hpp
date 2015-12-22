#ifndef ROS_YAML_IO_HPP
#define ROS_YAML_IO_HPP

/// PROJECT
#include <csapex/msg/generic_pointer_message.hpp>

/// SYSTEM
#include <yaml-cpp/node/convert.h>
#include <yaml-cpp/stlemitter.h>
#include <ros/serialization.h>

namespace csapex {
namespace serial {


template <typename Message>
struct Serializer<connection_types::GenericPointerMessage<Message>,
        typename std::enable_if<ros::message_traits::IsMessage<Message>::value>::type>
{
static YAML::Node encode(const connection_types::GenericPointerMessage<Message>& msg)
{
    YAML::Node n;

    // serialize the message using ros
    auto serialized = ros::serialization::serializeMessage(*msg.value);

    // allocate the buffer
    n["buf"] = std::vector<uint8_t>(serialized.num_bytes);
    auto buf = n["buf"];

    // fill the buffer
    for(std::size_t i = 0; i < serialized.num_bytes; ++i) {
        buf[i] = serialized.buf[i];
    }

    return n;
}

static bool decode(const YAML::Node& node, connection_types::GenericPointerMessage<Message>& msg)
{
    // read the buffer
    auto vec = node["buf"].as<std::vector<uint8_t>>();
    std::size_t num_bytes = vec.size();

    // ros needs a shared array instead of a vector
    boost::shared_array<uint8_t> buf(new uint8_t[num_bytes]);
    for(std::size_t i = 0; i < num_bytes; ++i) {
        buf[i] = vec[i];
    }

    // deserialize the message
    msg.value.reset(new Message);
    ros::serialization::deserializeMessage(ros::SerializedMessage(buf, num_bytes), *msg.value);
    return true;
}
};

}
}

#endif // ROS_YAML_IO_HPP
