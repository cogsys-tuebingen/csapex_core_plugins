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
    TransformMessage(const std::string &from_frame, const std::string &to_frame);

    virtual ConnectionType::Ptr clone() const override;

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
