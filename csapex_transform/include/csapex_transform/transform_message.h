#ifndef TRANSFORM_MESSAGE_H
#define TRANSFORM_MESSAGE_H

/// PROJECT
#include <csapex/msg/message_template.hpp>

/// SYSTEM
#include <tf/LinearMath/Transform.h>

namespace csapex {
namespace connection_types {


struct TransformMessage : public MessageTemplate<tf::Transform, TransformMessage>
{
    TransformMessage();
    TransformMessage(const std::string &frame_id, const std::string &child_frame_id);

    virtual ConnectionType::Ptr clone();

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

/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::TransformMessage> {
  static Node encode(const csapex::connection_types::TransformMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::TransformMessage& rhs);
};
}


#endif // TRANSFORM_MESSAGE_H
