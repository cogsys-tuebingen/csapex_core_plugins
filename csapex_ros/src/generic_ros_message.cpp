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
    node["type"] = rhs.value->getDataType();
    node["md5"] = rhs.value->getMD5Sum();

    auto serialized = ros::serialization::serializeMessage(*rhs.value);

    // allocate the buffer
    node["buf"] = std::vector<uint8_t>(serialized.num_bytes);
    auto buf = node["buf"];

    // fill the buffer
    for(std::size_t i = 0; i < serialized.num_bytes; ++i) {
        buf[i] = serialized.buf[i];
    }


    return node;
}

bool convert<csapex::connection_types::GenericRosMessage>::decode(const Node& node, csapex::connection_types::GenericRosMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }

    convert<csapex::connection_types::Message>::decode(node, rhs);

    // read the buffer
    auto vec = node["buf"].as<std::vector<uint8_t>>();
    int num_bytes = vec.size();

    // deserialize the message
    std::shared_ptr<topic_tools::ShapeShifter> shapeshifter = std::make_shared<topic_tools::ShapeShifter>();

    ros::serialization::IStream stream(vec.data(), num_bytes);
    ros::serialization::Serializer<topic_tools::ShapeShifter>::read(stream, *shapeshifter);

    rhs.value = shapeshifter;
    return true;
}
}
