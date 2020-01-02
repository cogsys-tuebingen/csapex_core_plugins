#ifndef ROS_BINARY_IO_HPP
#define ROS_BINARY_IO_HPP

/// PROJECT
#include <csapex/serialization/serialization_buffer.h>

/// SYSTEM
// clang-format off
#include <csapex/utility/suppress_warnings_start.h>
#include <ros/serialization.h>
#include <type_traits>
#include <csapex/utility/suppress_warnings_end.h>
// clang-format on

namespace csapex
{
template <typename Message, typename std::enable_if<ros::message_traits::IsMessage<Message>::value, int>::type = 0>
SerializationBuffer& operator<<(SerializationBuffer& data, const Message& rhs)
{
    ros::SerializedMessage serialized = ros::serialization::serializeMessage(rhs);

    data << serialized.num_bytes;
    data.writeRaw(serialized.message_start, serialized.num_bytes);

    return data;
}
template <typename Message, typename std::enable_if<ros::message_traits::IsMessage<Message>::value, int>::type = 0>
const SerializationBuffer& operator>>(const SerializationBuffer& data, std::remove_const_t<Message>& rhs)
{
    std::size_t num_bytes;
    data >> num_bytes;

    uint8_t* data_ptr = const_cast<uint8_t*>(data.data()) + data.getPos();
    ros::SerializedMessage serialized(boost::shared_array<uint8_t>(data_ptr), num_bytes);
    ros::serialization::deserializeMessage(serialized, rhs);

    data.advance(num_bytes);

    return data;
}
}  // namespace csapex

#endif  // ROS_BINARY_IO_HPP
