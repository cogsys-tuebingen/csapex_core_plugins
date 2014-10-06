#ifndef TIME_STAMP_MESSAGE_H
#define TIME_STAMP_MESSAGE_H

/// PROJECT
#include <csapex/msg/message_template.hpp>

/// SYSTEM
#include <ros/time.h>

namespace csapex {
namespace connection_types {


struct TimeStampMessage : public MessageTemplate<ros::Time, TimeStampMessage>
{
    TimeStampMessage();
};

/// TRAITS
template <>
struct type<TimeStampMessage> {
    static std::string name() {
        return "TimeStamp";
    }
};

}
}


/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::TimeStampMessage> {
  static Node encode(const csapex::connection_types::TimeStampMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::TimeStampMessage& rhs);
};
}

#endif // TIME_STAMP_MESSAGE_H
