#ifndef DURATION_MESSAGE_H
#define DURATION_MESSAGE_H

/// PROJECT
#include <csapex/msg/message_template.hpp>

/// SYSTEM
#include <chrono>

namespace csapex {
namespace connection_types {

struct DurationMessage : public MessageTemplate<std::chrono::microseconds, DurationMessage>
{
public:
    DurationMessage(std::chrono::microseconds duration = std::chrono::microseconds(0),
                    Message::Stamp stamp_micro_seconds = 0);
};


/// TRAITS
template <>
struct type<DurationMessage> {
    static std::string name() {
        return "Duration";
    }
};

template <>
inline std::shared_ptr<DurationMessage> makeEmpty<DurationMessage>()
{
    return std::shared_ptr<DurationMessage>(new DurationMessage(std::chrono::microseconds(0)));
}

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
