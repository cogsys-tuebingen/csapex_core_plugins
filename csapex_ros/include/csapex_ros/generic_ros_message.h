#ifndef GENERIC_ROS_MESSAGE_H
#define GENERIC_ROS_MESSAGE_H


/// PROJECT
#include <csapex/msg/message_template.hpp>

/// SYSTEM
#include <ros/time.h>

namespace topic_tools {
class ShapeShifter;
}

namespace csapex {
namespace connection_types {


struct GenericRosMessage : public MessageTemplate<std::shared_ptr<topic_tools::ShapeShifter const>, GenericRosMessage>
{
    GenericRosMessage();
};


/// TRAITS
template <>
struct type<GenericRosMessage> {
    static std::string name() {
        return "GenericRosMessage";
    }
};

}
}


/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::GenericRosMessage> {
  static Node encode(const csapex::connection_types::GenericRosMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::GenericRosMessage& rhs);
};
}

#endif // GENERIC_ROS_MESSAGE_H

