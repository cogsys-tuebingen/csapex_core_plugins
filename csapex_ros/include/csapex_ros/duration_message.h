#ifndef DURATION_MESSAGE_H
#define DURATION_MESSAGE_H

/// PROJECT
#include <csapex/msg/message_template.hpp>

/// SYSTEM
#include <ros/time.h>

namespace csapex {
namespace connection_types {


struct RosDurationMessage : public MessageTemplate<ros::Duration, RosDurationMessage>
{
    RosDurationMessage();
};


/// TRAITS
template <>
struct type<RosDurationMessage> {
    static std::string name() {
        return "Duration (ROS)";
    }
};

}
}


/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::RosDurationMessage> {
  static Node encode(const csapex::connection_types::RosDurationMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::RosDurationMessage& rhs);
};
}

#endif // DURATION_MESSAGE_H
