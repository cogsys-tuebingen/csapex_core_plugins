#ifndef ROS_YAML_IO_HPP
#define ROS_YAML_IO_HPP

/// PROJECT
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex/utility/yaml_io.hpp>
#include <csapex/serialization/node_serializer.h>

/// SYSTEM
#include <yaml-cpp/stlemitter.h>
#include <csapex/utility/suppress_warnings_start.h>
    #include <ros/serialization.h>
#include <csapex/utility/suppress_warnings_end.h>

namespace YAML
{
// ros message by value
template<typename Message>
struct convertPtr<Message, typename std::enable_if<ros::message_traits::IsMessage<Message>::value>::type> {
 static Node encode(const std::shared_ptr<Message>& msg) {
     YAML::Node n;

     // serialize the message using ros
     auto serialized = ros::serialization::serializeMessage(*msg);

     // allocate the buffer
     n["buf"] = std::vector<uint8_t>(serialized.num_bytes);
     auto buf = n["buf"];

     // fill the buffer
     for(std::size_t i = 0; i < serialized.num_bytes; ++i) {
         buf[i] = serialized.buf[i];
     }

     return n;
 }

 static bool decode(const Node& node, std::shared_ptr<Message>& msg) {
     // read the buffer
     auto vec = node["buf"].as<std::vector<uint8_t>>();
     std::size_t num_bytes = vec.size();

     // ros needs a shared array instead of a vector
     boost::shared_array<uint8_t> buf(new uint8_t[num_bytes]);
     for(std::size_t i = 0; i < num_bytes; ++i) {
         buf[i] = vec[i];
     }

     // deserialize the message
     msg.reset(new Message);
     ros::serialization::deserializeMessage(ros::SerializedMessage(buf, num_bytes), *msg);
     return true;
 }
};
template<typename Message>
struct convertConstPtr<Message, typename std::enable_if<ros::message_traits::IsMessage<Message>::value>::type> {
 static Node encode(const std::shared_ptr<Message const>& msg) {
     YAML::Node n;

     // serialize the message using ros
     auto serialized = ros::serialization::serializeMessage(*msg);

     // allocate the buffer
     n["buf"] = std::vector<uint8_t>(serialized.num_bytes);
     auto buf = n["buf"];

     // fill the buffer
     for(std::size_t i = 0; i < serialized.num_bytes; ++i) {
         buf[i] = serialized.buf[i];
     }

     return n;
 }

 static bool decode(const Node& node, std::shared_ptr<Message const>& msg) {
     // read the buffer
     auto vec = node["buf"].as<std::vector<uint8_t>>();
     std::size_t num_bytes = vec.size();

     // ros needs a shared array instead of a vector
     boost::shared_array<uint8_t> buf(new uint8_t[num_bytes]);
     for(std::size_t i = 0; i < num_bytes; ++i) {
         buf[i] = vec[i];
     }

     // deserialize the message
     std::shared_ptr<Message> tmp;
     tmp.reset(new Message);
     ros::serialization::deserializeMessage(ros::SerializedMessage(buf, num_bytes), *tmp);
     msg = tmp;
     return true;
 }
};
}


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
    std::shared_ptr<typename std::remove_const<Message>::type> tmp;
    tmp.reset(new typename std::remove_const<Message>::type);
    ros::serialization::deserializeMessage(ros::SerializedMessage(buf, num_bytes), *tmp);
    msg.value = tmp;
    return true;
}
};

}
}

#endif // ROS_YAML_IO_HPP
