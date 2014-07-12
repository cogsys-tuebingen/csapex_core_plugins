#ifndef TIME_STAMP_MESSAGE_H
#define TIME_STAMP_MESSAGE_H

/// PROJECT
#include <csapex/model/message.h>

/// SYSTEM
#include <ros/time.h>

namespace csapex {
namespace connection_types {


struct TimeStampMessage : public MessageTemplate<ros::Time, TimeStampMessage>
{
    TimeStampMessage();

    void writeYaml(YAML::Emitter& yaml) const;
    void readYaml(const YAML::Node& node);
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

#endif // TIME_STAMP_MESSAGE_H
