#ifndef DURATION_MESSAGE_H
#define DURATION_MESSAGE_H

/// PROJECT
#include <csapex/msg/message_template.hpp>
#include <csapex_core_plugins/csapex_core_lib_export.h>

/// SYSTEM
#include <chrono>

namespace csapex {
namespace connection_types {

struct CSAPEX_CORE_LIB_EXPORT DurationMessage : public MessageTemplate<std::chrono::microseconds, DurationMessage>
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

/// SERIALIZATION
SerializationBuffer& operator << (SerializationBuffer& data, const std::chrono::microseconds& t);
const SerializationBuffer& operator >> (const SerializationBuffer& data, std::chrono::microseconds& t);
}

/// YAML
namespace YAML {
template<>
struct CSAPEX_CORE_LIB_EXPORT convert<csapex::connection_types::DurationMessage> {
  static Node encode(const csapex::connection_types::DurationMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::DurationMessage& rhs);
};
}

#endif // DURATION_MESSAGE_H
