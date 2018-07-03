#ifndef TUTORIAL_MESSAGE_H
#define TUTORIAL_MESSAGE_H

/// PROJECT
#include <csapex/msg/message_template.hpp>

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace csapex {
namespace connection_types {

struct TutorialMessage : public MessageTemplate<bool, TutorialMessage>
{
    typedef std::shared_ptr<TutorialMessage> Ptr;
    typedef std::shared_ptr<TutorialMessage const> ConstPtr;

    TutorialMessage();

    void serialize(SerializationBuffer &data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;
};

/// TRAITS
template <>
struct type<TutorialMessage> {
    static std::string name() {
        return "TutorialMessage";
    }
};
}
}

/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::TutorialMessage> {
  static Node encode(const csapex::connection_types::TutorialMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::TutorialMessage& rhs);
};
}
#endif // TUTORIAL_MESSAGE_H
