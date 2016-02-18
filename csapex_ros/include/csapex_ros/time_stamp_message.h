#ifndef TIME_STAMP_MESSAGE_H
#define TIME_STAMP_MESSAGE_H

/// PROJECT
#include <csapex/msg/message_template.hpp>

/// SYSTEM
#include <ros/time.h>

namespace csapex {
namespace connection_types {


struct RosTimeStampMessage : public MessageTemplate<ros::Time, RosTimeStampMessage>
{
    RosTimeStampMessage();
};

/// TRAITS
template <>
struct type<RosTimeStampMessage> {
    static std::string name() {
        return "TimeStamp";
    }
};

}
}


/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::RosTimeStampMessage> {
  static Node encode(const csapex::connection_types::RosTimeStampMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::RosTimeStampMessage& rhs);
};
}

#endif // TIME_STAMP_MESSAGE_H
