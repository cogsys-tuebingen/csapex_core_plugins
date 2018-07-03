#ifndef ROS_BINARY_IO_HPP
#define ROS_BINARY_IO_HPP

/// PROJECT
#include <csapex/serialization/serialization_buffer.h>

/// SYSTEM
#include <csapex/utility/suppress_warnings_start.h>
    #include <ros/serialization.h>
#include <csapex/utility/suppress_warnings_end.h>

namespace csapex
{

template<typename Message, typename std::enable_if<ros::message_traits::IsMessage<Message>::value, int>::type = 0>
SerializationBuffer& operator << (SerializationBuffer& data, const Message& rhs)
{
    auto serialized = ros::serialization::serializeMessage(rhs);

    data << serialized.num_bytes;
    data.writeRaw(serialized.message_start, serialized.num_bytes);

    return data;
}
template<typename Message, typename std::enable_if<ros::message_traits::IsMessage<Message>::value, int>::type = 0>
const SerializationBuffer& operator >> (const SerializationBuffer& data, Message& rhs)
{
    std::size_t num_bytes;
    data >> num_bytes;

    uint8_t* data_ptr = const_cast<uint8_t*>(data.data()) + data.getPos();
    ros::serialization::IStream stream(data_ptr, num_bytes);
    ros::serialization::Serializer<Message>::read(stream, rhs);

    data.advance(num_bytes);

    return data;
}
}

#endif // ROS_BINARY_IO_HPP
