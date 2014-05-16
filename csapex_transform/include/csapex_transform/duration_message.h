#ifndef DURATION_MESSAGE_H
#define DURATION_MESSAGE_H

/// PROJECT
#include <csapex/model/message.h>

/// SYSTEM
#include <ros/time.h>

namespace csapex {
namespace connection_types {


struct DurationMessage : public MessageTemplate<ros::Duration, DurationMessage>
{
    DurationMessage();

    void writeYaml(YAML::Emitter& yaml);
    void readYaml(const YAML::Node& node);
};

}
}

#endif // DURATION_MESSAGE_H
