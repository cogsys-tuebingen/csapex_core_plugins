#ifndef CONFUSION_MATRIX_MESSAGE_H
#define CONFUSION_MATRIX_MESSAGE_H

/// COMPONENT
#include <csapex_evaluation/confusion_matrix.h>

/// PROJECT
#include <csapex/msg/message_template.hpp>

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace csapex {
namespace connection_types {


struct ConfusionMatrixMessage : public Message
{
    typedef boost::shared_ptr<ConfusionMatrixMessage> Ptr;

    ConfusionMatrixMessage(Message::Stamp stamp = 0);

    virtual ConnectionType::Ptr clone();
    virtual ConnectionType::Ptr toType();

    ConfusionMatrix confusion;
};

/// TRAITS
template <>
struct type<ConfusionMatrixMessage> {
    static std::string name() {
        return "ConfusionMatrixMessage";
    }
};
}
}

/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::ConfusionMatrixMessage> {
  static Node encode(const csapex::connection_types::ConfusionMatrixMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::ConfusionMatrixMessage& rhs);
};
}

#endif // CONFUSION_MATRIX_MESSAGE_H
