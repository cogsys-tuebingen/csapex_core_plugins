#ifndef TRANSFORM_MESSAGE_H
#define TRANSFORM_MESSAGE_H

/// PROJECT
#include <csapex/model/message.h>

/// SYSTEM
#include <tf/LinearMath/Transform.h>

namespace csapex {
namespace connection_types {


struct TransformMessage : public MessageTemplate<tf::Transform, TransformMessage>
{
    TransformMessage();
    TransformMessage(const std::string &from_frame, const std::string &to_frame);

    virtual ConnectionType::Ptr clone();

    void writeYaml(YAML::Emitter& yaml) const;
    void readYaml(const YAML::Node& node);

public:
    std::string child_frame;
};


/// TRAITS
template <>
struct type<TransformMessage> {
    static std::string name() {
        return "TransformMessage";
    }
};

}
}

#endif // TRANSFORM_MESSAGE_H
