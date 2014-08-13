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

    void writeYaml(YAML::Emitter& yaml) const;
    void readYaml(const YAML::Node& node);
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

#endif // DURATION_MESSAGE_H
