#ifndef DURATION_MESSAGE_H
#define DURATION_MESSAGE_H

/// PROJECT
#include <csapex/msg/message_template.hpp>

/// SYSTEM
#include <ros/time.h>

namespace csapex {
namespace connection_types {


struct DurationMessage : public MessageTemplate<ros::Duration, DurationMessage>
{
    DurationMessage();
};


/// TRAITS
template <>
struct type<DurationMessage> {
    static std::string name() {
        return "Duration";
    }
};

}
}


/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::DurationMessage> {
  static Node encode(const csapex::connection_types::DurationMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::DurationMessage& rhs);
};
}

#endif // DURATION_MESSAGE_H
